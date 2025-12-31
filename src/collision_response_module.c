// collision_response_module.c
#define _GNU_SOURCE
#include "collision_response_module.h"

#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

// 너가 만든 can_interface 사용
#include "can_interface_module.h"   // CANIF_send_brakelight()
                             // canif_brake_mode_t (BRAKE_YIELD)

// ----------------- 내부 시간 유틸 -----------------
static uint32_t now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(ts.tv_sec * 1000u + ts.tv_nsec / 1000000u);
}

// 0~359도에서 각도 차이(최소 각거리) 계산
static uint16_t ang_diff_deg(uint16_t a, uint16_t b)
{
    uint16_t d = (a > b) ? (a - b) : (b - a);
    return (d > 180) ? (360 - d) : d;
}

// ----------------- Door Unlock (stub) -----------------
// 실제 구현:
// - GPIO 제어, 릴레이, 솔레노이드, CAN으로 다른 ECU에 요청 등.
// 지금은 "요청만 발생"시키고, 나중에 hw 모듈로 교체.
static void door_unlock_pulse(uint32_t pulse_ms, int enable_stdout)
{
    if (enable_stdout) {
        printf("[CRESP] Door unlock pulse %ums\n", (unsigned)pulse_ms);
        fflush(stdout);
    }
    // TODO: 실제 GPIO/릴레이 제어
    (void)pulse_ms;
}

// ----------------- 상태/스레드 -----------------
static pthread_t g_thr;
static pthread_mutex_t g_mtx = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t  g_cv  = PTHREAD_COND_INITIALIZER;

static int g_running = 0;

static cresp_config_t g_cfg = {
    .heading_swing_threshold_deg = 30,
    .crash_cooldown_ms = 1000,
    .door_unlock_pulse_ms = 300,
    .enable_stdout = 1,
};

static cresp_handlers_t g_h = {0};

// 최근 heading (driving_info_module -> 여기로 feed)
static uint16_t g_heading_deg = 0;
static uint32_t g_heading_ts  = 0;

// collision 이벤트 시퀀스(consumer가 처리해야 할 이벤트 존재 여부)
static uint32_t g_collision_seq = 0;
static uint32_t g_seen_collision_seq = 0;

// 충돌 처리 중복 방지용
static uint32_t g_last_crash_ms = 0;

// 충돌 시점의 기준 heading 및 1초 윈도우에서 최대 흔들림 추적
static uint16_t g_crash_heading_base = 0;
static uint32_t g_crash_window_start_ms = 0;
static uint16_t g_crash_max_swing_deg = 0;
static int      g_crash_window_active = 0;
static int      g_sev3_sent = 0;

// severity 전달
static void emit_severity(uint8_t sev)
{
    if (g_cfg.enable_stdout) {
        printf("[CRESP] severity emit: %u\n", (unsigned)sev);
        fflush(stdout);
    }
    if (g_h.on_severity) g_h.on_severity(sev);
}

// CAN 브레이크 3단계(= BRAKE_YIELD: 4) 전송
static void send_brake_stage3(void)
{
    // time_ms는 yocto 기준 ms를 넣고 싶으면 now_ms()를 잘라서 쓰면 됨.
    // 너는 2바이트 time 필드를 써서 wrap 허용이었음.
    uint16_t t16 = (uint16_t)(now_ms() & 0xFFFFu);
    (void)CANIF_send_brakelight(BRAKE_YIELD, t16);
}

static void* worker(void* arg)
{
    (void)arg;

    pthread_mutex_lock(&g_mtx);
    while (g_running) {
        // 1) collision 이벤트가 올 때까지 대기
        while (g_running && g_collision_seq == g_seen_collision_seq) {
            pthread_cond_wait(&g_cv, &g_mtx);
        }
        if (!g_running) break;

        // 이벤트 처리 시작: seq 스냅샷
        g_seen_collision_seq = g_collision_seq;

        uint32_t t_now = now_ms();

        // 쿨다운: 너무 자주 들어오면 무시
        if (g_last_crash_ms && (t_now - g_last_crash_ms) < g_cfg.crash_cooldown_ms) {
            if (g_cfg.enable_stdout) {
                printf("[CRESP] collision ignored (cooldown)\n");
                fflush(stdout);
            }
            continue;
        }
        g_last_crash_ms = t_now;

        // 충돌 시점 기준 heading 확정
        g_crash_heading_base = g_heading_deg;
        g_crash_window_start_ms = t_now;
        g_crash_max_swing_deg = 0;
        g_crash_window_active = 1;
        g_sev3_sent = 0;

        // ---- 요구사항 2~4 ----
        pthread_mutex_unlock(&g_mtx);

        // 2) 차문 잠금 해제
        door_unlock_pulse(g_cfg.door_unlock_pulse_ms, g_cfg.enable_stdout);

        // 3) CAN) 브레이크등 3단계 전달
        send_brake_stage3();

        // 4) 사고 규모 2 전파
        emit_severity(2);

        pthread_mutex_lock(&g_mtx);

        // ---- 요구사항 5: 1초 동안 흔들림 체크 ----
        // 여기서 busy wait 하지 말고, heading 업데이트가 들어올 때마다 swing을 갱신하고
        // 1초가 지나면 sev3 여부를 결정한다.
        // 즉, 이 스레드는 "타임아웃"을 걸고 깨어나서 마감 처리한다.
        while (g_running && g_crash_window_active) {
            uint32_t elapsed = now_ms() - g_crash_window_start_ms;
            if (elapsed >= 1000u) {
                // 1초 종료
                if (!g_sev3_sent && g_crash_max_swing_deg >= g_cfg.heading_swing_threshold_deg) {
                    pthread_mutex_unlock(&g_mtx);
                    emit_severity(3);
                    pthread_mutex_lock(&g_mtx);
                    g_sev3_sent = 1;
                }
                g_crash_window_active = 0;
                break;
            }

            // heading 업데이트/stop 신호를 기다리되,
            // 1초 타임아웃까지 남은 만큼만 기다림
            struct timespec ts;
            clock_gettime(CLOCK_MONOTONIC, &ts);
            uint32_t remain_ms = 1000u - elapsed;
            ts.tv_sec  += remain_ms / 1000u;
            ts.tv_nsec += (long)(remain_ms % 1000u) * 1000000L;
            if (ts.tv_nsec >= 1000000000L) {
                ts.tv_sec += 1;
                ts.tv_nsec -= 1000000000L;
            }
            pthread_cond_timedwait(&g_cv, &g_mtx, &ts);
        }
    }
    pthread_mutex_unlock(&g_mtx);
    return NULL;
}

// ----------------- Public API -----------------
int CRESP_init(const cresp_config_t* cfg, const cresp_handlers_t* h)
{
    pthread_mutex_lock(&g_mtx);

    if (cfg) g_cfg = *cfg;
    if (h)   g_h   = *h;

    g_heading_deg = 0;
    g_heading_ts  = 0;

    g_collision_seq = 0;
    g_seen_collision_seq = 0;

    g_last_crash_ms = 0;
    g_crash_heading_base = 0;
    g_crash_window_start_ms = 0;
    g_crash_max_swing_deg = 0;
    g_crash_window_active = 0;
    g_sev3_sent = 0;

    pthread_mutex_unlock(&g_mtx);
    return 0;
}

int CRESP_start(void)
{
    pthread_mutex_lock(&g_mtx);
    if (g_running) { pthread_mutex_unlock(&g_mtx); return 0; }
    g_running = 1;
    pthread_mutex_unlock(&g_mtx);

    if (pthread_create(&g_thr, NULL, worker, NULL) != 0) {
        pthread_mutex_lock(&g_mtx);
        g_running = 0;
        pthread_mutex_unlock(&g_mtx);
        return -1;
    }
    return 0;
}

void CRESP_stop(void)
{
    pthread_mutex_lock(&g_mtx);
    if (!g_running) { pthread_mutex_unlock(&g_mtx); return; }
    g_running = 0;
    pthread_cond_broadcast(&g_cv);
    pthread_mutex_unlock(&g_mtx);

    pthread_join(g_thr, NULL);
}

void CRESP_on_collision_sensor(void)
{
    // ISR/콜백에서 호출된다고 가정: 짧게(lock + seq++ + signal)만.
    pthread_mutex_lock(&g_mtx);
    g_collision_seq++;
    pthread_cond_signal(&g_cv);
    pthread_mutex_unlock(&g_mtx);

    if (g_cfg.enable_stdout) {
        printf("[CRESP] collision sensor event\n");
        fflush(stdout);
    }
}

void CRESP_on_heading_deg(uint16_t heading_deg)
{
    if (heading_deg >= 360) heading_deg %= 360;

    pthread_mutex_lock(&g_mtx);
    g_heading_deg = heading_deg;
    g_heading_ts  = now_ms();

    // 충돌 윈도우 활성 중이면 흔들림 최대값 갱신
    if (g_crash_window_active) {
        uint16_t swing = ang_diff_deg(g_crash_heading_base, g_heading_deg);
        if (swing > g_crash_max_swing_deg) g_crash_max_swing_deg = swing;

        // 1초 끝나기 전에 이미 threshold 넘으면 즉시 sev3를 쏠 수도 있다.
        // 요구사항은 "1000ms 동안 크게 흔들린다면"이라서,
        // 여기서는 threshold 초과를 기록만 하고 최종 판단은 worker가 1초 끝에 한다.
    }

    pthread_cond_signal(&g_cv);
    pthread_mutex_unlock(&g_mtx);
}
