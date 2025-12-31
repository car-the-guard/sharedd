// accident_announce_module.h
#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 3차선 기준: 1,2,3 (값 그대로 CAN/SPI payload에 실어도 됨)
typedef enum {
    LANE_1 = 1,
    LANE_2 = 2,
    LANE_3 = 3,
    LANE_INVALID = 255
} lane_t;

// 규모(원하는대로 바꿔도 됨)
typedef enum {
    SEV_1 = 1,
    SEV_2 = 2,
    SEV_3 = 3
} severity_t;

typedef struct {
    uint32_t ts_ms;      // 수신 시각(모듈 내부에서 찍음)
    uint16_t dist_m;     // 거리(m). cm 쓰면 dist_cm로 바꾸고 단위 고정.
    lane_t lane;         // 1/2/3
    severity_t sev;      // 1/2/3
    uint32_t seq;        // 갱신 카운터(모듈 내부)
} accident_info_t;

// 옵션
typedef struct {
    uint32_t min_announce_interval_ms; // 동일/유사 사고 재안내 최소 간격
    uint16_t dist_bucket_m;            // 거리 버킷(예: 5m 단위)
    int enable_stdout;                 // 1이면 printf로 출력
} aann_config_t;

// init/start/stop
int  AANN_init(const aann_config_t* cfg);
int  AANN_start(void);
void AANN_stop(void);

// SPI 모듈이 "콜백"으로 호출하는 함수 (ISR처럼 가볍게 써야 함)
void AANN_on_accident(uint16_t dist_m, lane_t lane, severity_t sev);

// (선택) 최신값 조회
int  AANN_get_latest(accident_info_t* out);

#ifdef __cplusplus
}
#endif
