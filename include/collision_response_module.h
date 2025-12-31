// collision_response_module.h
#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ---- 외부 모듈 의존(연결부) ----
// can_interface.h 를 이미 갖고 있으니, 여기서는 함수만 사용한다고 가정.
// - BRAKE_YIELD(4) = "3단계"로 쓰겠다(네가 정의한 0/1/2/4 그대로).
// - AEB/모터제어는 여기서 직접 안 건드리고(이미 risk 쪽에서), "충돌 이후 대응"만 한다.

typedef struct {
    // 충돌 이후 1초 동안 "방향 흔들림" 판정 임계값 (deg)
    uint16_t heading_swing_threshold_deg;   // 예: 30

    // 충돌 이벤트 디바운스/쿨다운 (ms)
    uint32_t crash_cooldown_ms;             // 예: 1000

    // 문 잠금 해제 유지 시간(ms) (하드웨어에 따라 필요하면)
    uint32_t door_unlock_pulse_ms;          // 예: 300

    // 로그 출력
    int enable_stdout;
} cresp_config_t;

// severity 전파 콜백 (사고 전파 모듈로 던질 것)
typedef void (*cresp_on_severity_fn)(uint8_t severity); // 2 or 3

typedef struct {
    cresp_on_severity_fn on_severity; // NULL 가능
} cresp_handlers_t;

// 라이프사이클
int  CRESP_init(const cresp_config_t* cfg, const cresp_handlers_t* h);
int  CRESP_start(void);
void CRESP_stop(void);

// 입력 이벤트(다른 모듈/인터럽트에서 호출)
// 1) 충돌 감지 센서 신호(진동/충돌)
void CRESP_on_collision_sensor(void);

// 2) 지자기 heading 업데이트(0~359). driving_info_module에서 호출해도 됨.
void CRESP_on_heading_deg(uint16_t heading_deg);

#ifdef __cplusplus
}
#endif
