// // collision_risk_module.c
// #define _GNU_SOURCE
// #include "collision_risk_module.h"
// #include <pthread.h>
// #include <string.h>
// #include <time.h>
// #include <math.h>
// #include <unistd.h>

// static pthread_t g_thr;
// static pthread_mutex_t g_mtx = PTHREAD_MUTEX_INITIALIZER;
// static int g_running = 0;

// static crm_config_t g_cfg;
// static crm_callbacks_t g_cb;

// static crm_inputs_t g_in;
// static crm_state_t  g_st;

// // 이벤트 스팸 방지
// static uint32_t g_last_evt_ms_brake1 = 0;
// static uint32_t g_last_evt_ms_aeb    = 0;
// static uint32_t g_last_evt_ms_brake2 = 0;
// static uint32_t g_last_evt_ms_scale1 = 0;
// static uint32_t g_last_evt_ms_airbag = 0;

// static uint32_t now_ms(void)
// {
//     struct timespec ts;
//     clock_gettime(CLOCK_MONOTONIC, &ts);
//     return (uint32_t)(ts.tv_sec * 1000u + ts.tv_nsec / 1000000u);
// }

// static int can_fire(uint32_t* last_ms)
// {
//     uint32_t t = now_ms();
//     if (*last_ms == 0 || (t - *last_ms) >= g_cfg.min_event_interval_ms) {
//         *last_ms = t;
//         return 1;
//     }
//     return 0;
// }

// // dist + v*t + 0.5*a*t^2 = 0 의 "양의 최소해"를 구한다.
// // - dist_m: 전방까지 남은 거리 (>=0)
// // - v_mps : 전방을 향하는 상대속도(>0이면 다가감). 여기서는 speed_mps를 그대로 쓰되,
// //          너희 시스템에서 "상대속도"를 계산할 수 있으면 그걸 넣는 게 맞다.
// // - a_mps2: 전방을 향하는 상대가속도(>0이면 더 빨라짐, <0이면 감속)
// // 반환: ttc_ms (0이면 즉시/충돌), UINT32_MAX이면 충돌 안함(해 없음)
// static uint32_t compute_ttc_ms(float dist_m, float v_mps, float a_mps2)
// {
//     if (dist_m <= 0.0f) return 0;

//     // 전방 객체가 멀어지거나 정지 상태(다가가지 않음)면 충돌 없음 처리
//     // (상대속도 기준으로 쓰는 게 핵심)
//     if (v_mps <= 0.001f && a_mps2 <= 0.0f) return UINT32_MAX;

//     // a가 거의 0이면 선형 근사
//     if (fabsf(a_mps2) < 1e-4f) {
//         if (v_mps <= 0.001f) return UINT32_MAX;
//         float t = dist_m / v_mps;
//         if (t < 0.0f) return UINT32_MAX;
//         float ms = t * 1000.0f;
//         if (ms > 4.0e9f) return UINT32_MAX;
//         return (uint32_t)ms;
//     }

//     // 0.5*a*t^2 + v*t + dist = 0  (충돌 시 dist가 0이 되는 시점)
//     // 일반적으로 dist>0, v>0면 해가 음수로 나올 수 있어 sign convention 주의.
//     // 여기서는 "dist를 줄이는 방향"을 +로 보고, 충돌은 dist(t)=dist - (v*t + 0.5*a*t^2) = 0 형태가 직관적이다.
//     // 따라서: dist - v*t - 0.5*a*t^2 = 0  => 0.5*a*t^2 + v*t - dist = 0 (a는 전방방향 가속)
//     float A = 0.5f * a_mps2;
//     float B = v_mps;
//     float C = -dist_m;

//     float D = B*B - 4.0f*A*C;
//     if (D < 0.0f) return UINT32_MAX;

//     float sqrtD = sqrtf(D);

//     // 두 해 중 양의 최소해
//     float t1 = (-B + sqrtD) / (2.0f*A);
//     float t2 = (-B - sqrtD) / (2.0f*A);

//     float t = 1e30f;
//     if (t1 > 0.0f && t1 < t) t = t1;
//     if (t2 > 0.0f && t2 < t) t = t2;

//     if (t == 1e30f) return UINT32_MAX;

//     float ms = t * 1000.0f;
//     if (ms < 0.0f) return UINT32_MAX;
//     if (ms > 4.0e9f) return UINT32_MAX;
//     return (uint32_t)ms;
// }

// static void act_brake_level(int level, uint32_t* last_evt)
// {
//     if (!g_cb.send_brake_lamp_level) return;
//     if (!can_fire(last_evt)) return;
//     g_cb.send_brake_lamp_level(level);
// }

// static void act_aeb(int enable, uint32_t* last_evt)
// {
//     if (!g_cb.send_aeb) return;
//     if (!can_fire(last_evt)) return;
//     g_cb.send_aeb(enable);
// }

// static void act_scale1(uint32_t* last_evt)
// {
//     if (!g_cb.notify_accident_scale) return;
//     if (!can_fire(last_evt)) return;
//     g_cb.notify_accident_scale(1);
// }

// static void act_airbag(int enable, uint32_t* last_evt)
// {
//     if (!g_cb.deploy_airbag) return;
//     if (!can_fire(last_evt)) return;
//     g_cb.deploy_airbag(enable);
// }

// static void step_once(void)
// {
//     crm_inputs_t in;
//     pthread_mutex_lock(&g_mtx);
//     in = g_in;
//     pthread_mutex_unlock(&g_mtx);

//     // 객체 없으면 리스크 off로 수렴(여기서 정책 고정)
//     if (!in.object_present) {
//         pthread_mutex_lock(&g_mtx);
//         g_st.last_ttc_ms = UINT32_MAX;
//         g_st.brake_level = 0;
//         g_st.aeb_on = 0;
//         g_st.airbag_on = 0;
//         pthread_mutex_unlock(&g_mtx);
//         return;
//     }

//     uint32_t ttc_ms = compute_ttc_ms(in.dist_m, in.speed_mps, in.accel_mps2);

//     // 상태 갱신
//     pthread_mutex_lock(&g_mtx);
//     g_st.last_ttc_ms = ttc_ms;
//     pthread_mutex_unlock(&g_mtx);

//     if (ttc_ms == UINT32_MAX) {
//         // 위험 아님
//         pthread_mutex_lock(&g_mtx);
//         g_st.brake_level = 0;
//         g_st.aeb_on = 0;
//         g_st.airbag_on = 0;
//         pthread_mutex_unlock(&g_mtx);
//         return;
//     }

//     // 1) TTC 기반 브레이크등 1단계(경고)
//     if (ttc_ms <= g_cfg.ttc_brake1_ms && ttc_ms > g_cfg.ttc_aeb_ms) {
//         pthread_mutex_lock(&g_mtx);
//         g_st.brake_level = 1;
//         pthread_mutex_unlock(&g_mtx);
//         act_brake_level(1, &g_last_evt_ms_brake1);
//     }

//     // 2) AEB 구간: AEB + 브레이크등 2단계 + 사고규모1
//     if (ttc_ms <= g_cfg.ttc_aeb_ms) {
//         pthread_mutex_lock(&g_mtx);
//         g_st.aeb_on = 1;
//         g_st.brake_level = 2;
//         pthread_mutex_unlock(&g_mtx);

//         act_aeb(1, &g_last_evt_ms_aeb);
//         act_brake_level(2, &g_last_evt_ms_brake2);
//         act_scale1(&g_last_evt_ms_scale1);
//     }

//     // 3) AEB 이후에도 불가피(아주 작은 TTC)면 에어백 조건 체크
//     // 상대속도는 시스템이 진짜 상대속도를 넣어야 의미가 있음.
//     if (ttc_ms <= g_cfg.ttc_unavoidable_ms) {
//         // 여기선 speed_mps를 "상대속도"로 해석한다.
//         if (in.speed_mps >= g_cfg.rel_speed_airbag_mps) {
//             pthread_mutex_lock(&g_mtx);
//             g_st.airbag_on = 1;
//             pthread_mutex_unlock(&g_mtx);
//             act_airbag(1, &g_last_evt_ms_airbag);
//         }
//     }
// }

// static void* crm_thread(void* arg)
// {
//     (void)arg;

//     while (1) {
//         pthread_mutex_lock(&g_mtx);
//         int run = g_running;
//         pthread_mutex_unlock(&g_mtx);
//         if (!run) break;

//         step_once();
//         usleep(g_cfg.period_ms * 1000u);
//     }
//     return NULL;
// }

// int CRM_init(const crm_config_t* cfg, const crm_callbacks_t* cb)
// {
//     crm_config_t def = {
//         .period_ms = 50,

//         .ttc_brake1_ms = 1200,       // 경고(브레이크등 1단계)
//         .ttc_aeb_ms = 700,           // AEB
//         .ttc_unavoidable_ms = 250,   // 거의 충돌 확정으로 간주

//         .rel_speed_airbag_mps = 2.0f, // 2m/s 이상이면 에어백(임시값)
//         .min_event_interval_ms = 200  // 이벤트 스팸 방지
//     };

//     pthread_mutex_lock(&g_mtx);
//     g_cfg = cfg ? *cfg : def;
//     memset(&g_cb, 0, sizeof(g_cb));
//     if (cb) g_cb = *cb;

//     memset(&g_in, 0, sizeof(g_in));
//     g_in.object_present = 0;

//     memset(&g_st, 0, sizeof(g_st));
//     g_st.last_ttc_ms = UINT32_MAX;

//     g_last_evt_ms_brake1 = 0;
//     g_last_evt_ms_aeb    = 0;
//     g_last_evt_ms_brake2 = 0;
//     g_last_evt_ms_scale1 = 0;
//     g_last_evt_ms_airbag = 0;

//     pthread_mutex_unlock(&g_mtx);
//     return 0;
// }

// int CRM_start(void)
// {
//     pthread_mutex_lock(&g_mtx);
//     if (g_running) { pthread_mutex_unlock(&g_mtx); return 0; }
//     g_running = 1;
//     pthread_mutex_unlock(&g_mtx);

//     if (pthread_create(&g_thr, NULL, crm_thread, NULL) != 0) {
//         pthread_mutex_lock(&g_mtx);
//         g_running = 0;
//         pthread_mutex_unlock(&g_mtx);
//         return -1;
//     }
//     return 0;
// }

// void CRM_stop(void)
// {
//     pthread_mutex_lock(&g_mtx);
//     if (!g_running) { pthread_mutex_unlock(&g_mtx); return; }
//     g_running = 0;
//     pthread_mutex_unlock(&g_mtx);

//     pthread_join(g_thr, NULL);
// }

// void CRM_update_inputs(const crm_inputs_t* in)
// {
//     if (!in) return;
//     pthread_mutex_lock(&g_mtx);
//     g_in = *in;
//     pthread_mutex_unlock(&g_mtx);
// }

// int CRM_get_state(crm_state_t* out)
// {
//     if (!out) return -1;
//     pthread_mutex_lock(&g_mtx);
//     *out = g_st;
//     pthread_mutex_unlock(&g_mtx);
//     return 0;
// }

// collision_risk_module.c
#define _GNU_SOURCE
#include "collision_risk_module.h"
#include <pthread.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include <unistd.h>
#include <stdio.h> // 디버그 로그용

static pthread_t g_thr;
static pthread_mutex_t g_mtx = PTHREAD_MUTEX_INITIALIZER;
static int g_running = 0;

static crm_config_t g_cfg;
static crm_callbacks_t g_cb;

static crm_inputs_t g_in;
static crm_state_t  g_st;

// 이벤트 스팸 방지 타이머들
static uint32_t g_last_evt_ms_brake1 = 0;
static uint32_t g_last_evt_ms_aeb    = 0;
static uint32_t g_last_evt_ms_brake2 = 0;
static uint32_t g_last_evt_ms_scale1 = 0;
static uint32_t g_last_evt_ms_airbag = 0;

static uint32_t now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(ts.tv_sec * 1000u + ts.tv_nsec / 1000000u);
}

static int can_fire(uint32_t* last_ms)
{
    uint32_t t = now_ms();
    // 0이면 처음, 아니면 간격 체크
    if (*last_ms == 0 || (t - *last_ms) >= g_cfg.min_event_interval_ms) {
        *last_ms = t;
        return 1;
    }
    return 0;
}

// TTC 계산: 변수명만 내부적으로 매핑 (로직 유지)
// v_rel > 0: 다가옴, a_rel > 0: 다가오는 속도 증가
static uint32_t compute_ttc_ms(float dist_m, float v_rel_mps, float a_rel_mps2)
{
    if (dist_m <= 0.0f) return 0;

    // [수정] 멀어지거나(v<=0) 정지 상태면 충돌 없음
    if (v_rel_mps <= 0.001f && a_rel_mps2 <= 0.0f) return UINT32_MAX;

    // 등속 운동 근사 (가속도가 거의 0일 때)
    if (fabsf(a_rel_mps2) < 1e-4f) {
        if (v_rel_mps <= 0.001f) return UINT32_MAX;
        float t = dist_m / v_rel_mps;
        float ms = t * 1000.0f;
        if (ms > 4.0e9f) return UINT32_MAX; // 오버플로우 방지
        return (uint32_t)ms;
    }

    // 2차 방정식: 0.5*a*t^2 + v*t - dist = 0
    float A = 0.5f * a_rel_mps2;
    float B = v_rel_mps;
    float C = -dist_m;

    float D = B*B - 4.0f*A*C;
    if (D < 0.0f) return UINT32_MAX; // 허근(충돌 안함)

    float sqrtD = sqrtf(D);
    float t1 = (-B + sqrtD) / (2.0f*A);
    float t2 = (-B - sqrtD) / (2.0f*A);

    float t = 1e30f;
    if (t1 > 0.0f && t1 < t) t = t1;
    if (t2 > 0.0f && t2 < t) t = t2;

    if (t == 1e30f) return UINT32_MAX;

    float ms = t * 1000.0f;
    if (ms > 4.0e9f) return UINT32_MAX;
    return (uint32_t)ms;
}

// 액션 헬퍼 함수들 (기존 유지)
static void act_brake_level(int level, uint32_t* last_evt) {
    if (g_cb.send_brake_lamp_level && can_fire(last_evt)) 
        g_cb.send_brake_lamp_level(level);
}
static void act_aeb(int enable, uint32_t* last_evt) {
    if (g_cb.send_aeb && can_fire(last_evt)) 
        g_cb.send_aeb(enable);
}
static void act_scale1(uint32_t* last_evt) {
    if (g_cb.notify_accident_scale && can_fire(last_evt)) 
        g_cb.notify_accident_scale(1);
}
static void act_airbag(int enable, uint32_t* last_evt) {
    if (g_cb.deploy_airbag && can_fire(last_evt)) 
        g_cb.deploy_airbag(enable);
}

// [핵심 로직]
static void step_once(void)
{
    crm_inputs_t in;
    uint32_t now = now_ms();

    // 1. 입력 복사 (Atomic)
    pthread_mutex_lock(&g_mtx);
    in = g_in;
    pthread_mutex_unlock(&g_mtx);

    // [추가] 데이터 신선도 체크 (Failsafe)
    // 센서가 죽어서 마지막 값(충돌 직전 값)이 계속 남아있는 경우 방지
    uint32_t age = (now >= in.timestamp_ms) ? (now - in.timestamp_ms) : 0;
    
    if (age > g_cfg.max_data_age_ms) {
        // 데이터가 너무 오래됨 -> 모든 제어 해제 및 대기
        pthread_mutex_lock(&g_mtx);
        g_st.last_ttc_ms = UINT32_MAX;
        g_st.brake_level = 0;
        g_st.aeb_on = 0;
        g_st.airbag_on = 0;
        g_st.is_data_fresh = 0; // [추가] 상태 표시
        pthread_mutex_unlock(&g_mtx);
        
        // 필요하다면 여기서 "센서 에러" 로그 출력
        return; 
    }

    // 데이터 신선함 표시
    pthread_mutex_lock(&g_mtx);
    g_st.is_data_fresh = 1;
    pthread_mutex_unlock(&g_mtx);

    // 2. 객체 없음 -> 리셋
    if (!in.object_present) {
        pthread_mutex_lock(&g_mtx);
        g_st.last_ttc_ms = UINT32_MAX;
        g_st.brake_level = 0;
        g_st.aeb_on = 0;
        g_st.airbag_on = 0;
        pthread_mutex_unlock(&g_mtx);
        return;
    }

    // 3. TTC 계산 (상대속도, 상대가속도 사용)
    uint32_t ttc_ms = compute_ttc_ms(in.dist_m, in.rel_speed_mps, in.rel_accel_mps2);

    // 상태 저장
    pthread_mutex_lock(&g_mtx);
    g_st.last_ttc_ms = ttc_ms;
    pthread_mutex_unlock(&g_mtx);

    // 안전 상태
    if (ttc_ms == UINT32_MAX) {
        // 이전에 켜진게 있으면 꺼야 함 (단, AEB/에어백은 래치(Latch)될 수 있으나 여기선 즉시 해제 구조)
        // 실제 차에서는 AEB가 한번 걸리면 운전자가 엑셀 밟기 전까지 유지되는 등 복잡함. 여기선 단순화.
        act_aeb(0, &g_last_evt_ms_aeb); // 명시적 해제 전송
        act_brake_level(0, &g_last_evt_ms_brake1);
        
        pthread_mutex_lock(&g_mtx);
        g_st.brake_level = 0;
        g_st.aeb_on = 0;
        g_st.airbag_on = 0;
        pthread_mutex_unlock(&g_mtx);
        return;
    }

    // 4. 단계별 판단 로직
    int curr_brake = 0;
    int curr_aeb = 0;
    int curr_airbag = 0;

    // 단계 1: 경고
    if (ttc_ms <= g_cfg.ttc_brake1_ms) {
        curr_brake = 1;
        act_brake_level(1, &g_last_evt_ms_brake1);
    }

    // 단계 2: AEB (경고보다 우선순위 높으므로 덮어씀)
    if (ttc_ms <= g_cfg.ttc_aeb_ms) {
        curr_brake = 2; // 급제동 점멸
        curr_aeb = 1;
        
        act_aeb(1, &g_last_evt_ms_aeb);
        act_brake_level(2, &g_last_evt_ms_brake2);
        act_scale1(&g_last_evt_ms_scale1); // 사고 규모 알림
    }

    // 단계 3: 충돌 불가피 (Airbag / Pre-crash)
    if (ttc_ms <= g_cfg.ttc_unavoidable_ms) {
        // [수정] 상대 속도가 충분히 빠를 때만 작동 (주차장 등 저속 충돌 제외)
        if (in.rel_speed_mps >= g_cfg.rel_speed_airbag_mps) {
            curr_airbag = 1;
            act_airbag(1, &g_last_evt_ms_airbag);
        }
    }

    // 최종 상태 업데이트
    pthread_mutex_lock(&g_mtx);
    g_st.brake_level = curr_brake;
    g_st.aeb_on = curr_aeb;
    g_st.airbag_on = curr_airbag;
    pthread_mutex_unlock(&g_mtx);
}

static void* crm_thread(void* arg)
{
    (void)arg;
    while (1) {
        pthread_mutex_lock(&g_mtx);
        int run = g_running;
        pthread_mutex_unlock(&g_mtx);
        if (!run) break;

        step_once();
        usleep(g_cfg.period_ms * 1000u);
    }
    return NULL;
}

int CRM_init(const crm_config_t* cfg, const crm_callbacks_t* cb)
{
    crm_config_t def = {
        .period_ms = 50,
        .ttc_brake1_ms = 1500,       // 조금 더 여유 있게
        .ttc_aeb_ms = 800,           
        .ttc_unavoidable_ms = 300,   
        .rel_speed_airbag_mps = 3.0f, // 약 10km/h 이상 차이날 때
        .min_event_interval_ms = 100,
        .max_data_age_ms = 500       // [추가] 0.5초 동안 데이터 없으면 고장 간주
    };

    pthread_mutex_lock(&g_mtx);
    g_cfg = cfg ? *cfg : def;
    memset(&g_cb, 0, sizeof(g_cb));
    if (cb) g_cb = *cb;
    
    // 초기화 시 timestamp를 0으로 두어 "오래된 데이터" 상태로 시작
    memset(&g_in, 0, sizeof(g_in));
    g_in.timestamp_ms = 0; 

    memset(&g_st, 0, sizeof(g_st));
    g_st.last_ttc_ms = UINT32_MAX;
    
    // 이벤트 타이머 리셋
    g_last_evt_ms_brake1 = 0;
    // ... (나머지 0 초기화)
    
    pthread_mutex_unlock(&g_mtx);
    return 0;
}

int CRM_start(void)
{
    pthread_mutex_lock(&g_mtx);
    if (g_running) { pthread_mutex_unlock(&g_mtx); return 0; }
    g_running = 1;
    pthread_mutex_unlock(&g_mtx);

    if (pthread_create(&g_thr, NULL, crm_thread, NULL) != 0) {
        pthread_mutex_lock(&g_mtx);
        g_running = 0;
        pthread_mutex_unlock(&g_mtx);
        return -1;
    }
    return 0;
}

void CRM_stop(void)
{
    pthread_mutex_lock(&g_mtx);
    if (!g_running) { pthread_mutex_unlock(&g_mtx); return; }
    g_running = 0;
    pthread_mutex_unlock(&g_mtx);
    pthread_join(g_thr, NULL);
}

void CRM_update_inputs(const crm_inputs_t* in)
{
    if (!in) return;
    
    uint32_t t = now_ms();
    
    pthread_mutex_lock(&g_mtx);
    g_in = *in;
    g_in.timestamp_ms = t; // [수정] 들어온 시간 자동 기록
    pthread_mutex_unlock(&g_mtx);
}

int CRM_get_state(crm_state_t* out)
{
    if (!out) return -1;
    pthread_mutex_lock(&g_mtx);
    *out = g_st;
    pthread_mutex_unlock(&g_mtx);
    return 0;
}