// // collision_risk_module.h
// #pragma once
// #include <stdint.h>

// #ifdef __cplusplus
// extern "C" {
// #endif

// typedef struct {
//     // loop
//     uint32_t period_ms;                 // 판단 주기

//     // TTC 임계값(ms)
//     uint32_t ttc_brake1_ms;             // 브레이크등 1단계(경고)
//     uint32_t ttc_aeb_ms;                // AEB 트리거
//     uint32_t ttc_unavoidable_ms;        // "급정거 후에도 충돌 불가피" 판단 기준

//     // 상대속도 임계값 (0.00001 m/s 단위 등으로 고정하고 싶으면 바꿔라)
//     // 여기서는 m/s로 사용.
//     float rel_speed_airbag_mps;         // 이 이상이면 에어백 전개

//     // 이벤트 스팸 방지
//     uint32_t min_event_interval_ms;     // 동일 이벤트 재발행 최소 간격
// } crm_config_t;

// // 외부로 나가는 액션(다른 모듈 연결 지점)
// typedef struct {
//     // 브레이크등 단계: 0=off, 1=점등, 2=점멸(또는 더 강한 단계)
//     void (*send_brake_lamp_level)(int level);

//     // AEB on/off
//     void (*send_aeb)(int enable);

//     // 사고 전파 모듈로 규모 전달 (여기선 규모1만 쓰면 scale=1로 호출)
//     void (*notify_accident_scale)(int scale);

//     // 에어백 전개
//     void (*deploy_airbag)(int enable);
// } crm_callbacks_t;

// // 입력(주행정보 저장소에서 가져오거나, 이벤트로 밀어넣거나 둘 다 가능)
// typedef struct {
//     // 거리(m)
//     float dist_m;

//     // 내 차량 속도(m/s) - "전방 객체와의 상대 속도"로 쓰려면 rel_speed_mps에 바로 넣어라.
//     float speed_mps;

//     // 가속도(m/s^2) : 전방 방향(감속은 음수, 가속은 양수)
//     float accel_mps2;

//     // 전방 객체 인식 여부(0이면 TTC 계산 스킵)
//     int object_present;

//     // (선택) 차선/객체 같은 추가 신호가 필요하면 여기 확장
// } crm_inputs_t;

// int  CRM_init(const crm_config_t* cfg, const crm_callbacks_t* cb);
// int  CRM_start(void);
// void CRM_stop(void);

// // 입력 갱신: producer(센서/AI/속도 모듈)가 호출
// void CRM_update_inputs(const crm_inputs_t* in);

// // 상태 조회(테스트/로그)
// typedef struct {
//     uint32_t last_ttc_ms;
//     int brake_level;    // 0/1/2
//     int aeb_on;         // 0/1
//     int airbag_on;      // 0/1
// } crm_state_t;

// int  CRM_get_state(crm_state_t* out);

// #ifdef __cplusplus
// }
// #endif

// collision_risk_module.h
#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    // loop
    uint32_t period_ms;             // 판단 주기

    // TTC 임계값(ms)
    uint32_t ttc_brake1_ms;         // 브레이크등 1단계(경고)
    uint32_t ttc_aeb_ms;            // AEB 트리거
    uint32_t ttc_unavoidable_ms;    // 충돌 불가피 판단 기준

    // [수정] 물리량 기준 명확화
    // 상대속도 임계값 (m/s)
    float rel_speed_airbag_mps;     // 이 속도 이상으로 접근 중일 때만 에어백 고려

    // 이벤트 스팸 방지
    uint32_t min_event_interval_ms; 

    // [추가] 데이터 유효성 검사 시간 (이 시간 지나면 센서 고장으로 간주)
    uint32_t max_data_age_ms;       
} crm_config_t;

// 외부 액션 콜백 (변경 없음)
typedef struct {
    void (*send_brake_lamp_level)(int level);
    void (*send_aeb)(int enable);
    void (*notify_accident_scale)(int scale);
    void (*deploy_airbag)(int enable);
} crm_callbacks_t;

// 입력 구조체
typedef struct {
    // 거리(m)
    float dist_m;

    // [수정] 물리적 의미 명확화: "상대 속도"
    // 양수(+) : 서로 가까워지는 중 (충돌 위험)
    // 음수(-) : 멀어지는 중
    // 계산법: (내차속도 - 앞차속도) 또는 AI가 주는 접근 속도
    float rel_speed_mps;

    // [수정] "상대 가속도"
    // 양수(+) : 접근 속도가 더 빨라지는 중 (내가 가속하거나 앞차가 급브레이크)
    // 음수(-) : 접근 속도가 줄어드는 중 (내가 브레이크 밟음)
    float rel_accel_mps2;

    // 전방 객체 인식 여부
    int object_present;

    // [추가] 데이터 수신 시각 (모듈 내부에서 채워도 되고, 외부에서 줘도 됨)
    // 여기서는 update 함수 호출 시 내부적으로 기록하도록 구현 예정
    uint32_t timestamp_ms; 
} crm_inputs_t;

int  CRM_init(const crm_config_t* cfg, const crm_callbacks_t* cb);
int  CRM_start(void);
void CRM_stop(void);

// 입력 갱신
void CRM_update_inputs(const crm_inputs_t* in); // timestamp는 내부에서 찍음

// 상태 조회
typedef struct {
    uint32_t last_ttc_ms;
    int brake_level;
    int aeb_on;
    int airbag_on;
    
    // [추가] 디버깅용: 데이터가 유효한지(Fresh) 여부
    int is_data_fresh; 
} crm_state_t;

int  CRM_get_state(crm_state_t* out);

#ifdef __cplusplus
}
#endif