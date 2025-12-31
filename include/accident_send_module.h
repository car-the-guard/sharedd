// accident_send_module.h
#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ===== WL-2 (6B) 사고 요약 패킷 =====
// Byte0-1: Distance(12bit, m) <<4 | RSV(4bit)
// Byte2  : Lane (1/2/3)
// Byte3  : Severity(4bit)<<4 | RSV(4bit)
// Byte4-5: Timestamp (uint16_t, 10ms tick)
typedef struct __attribute__((packed)) {
    uint16_t dist_rsv;   // [15:4]=distance(m,0..4095), [3:0]=rsv
    uint8_t  lane;       // 1/2/3
    uint8_t  sev_rsv;    // [7:4]=severity(0..15), [3:0]=rsv
    uint16_t ts_10ms;    // 10ms tick (wrap OK)
} wl2_packet_t;

// 사고 종류(너희가 나중에 확장)
typedef enum {
    ACC_TYPE_UNKNOWN = 0,
    ACC_TYPE_COLLISION = 1,
    ACC_TYPE_HARD_BRAKE = 2,
    ACC_TYPE_OBSTACLE = 3,
} acc_type_t;

// 외부 모듈에서 공급해줘야 하는 “현재 주행 정보” 소스(= driving_info_module 역할)
// 이 모듈은 저장소를 직접 들고 있지 않고, 필요할 때 조회함(결합도 낮음).
typedef struct {
    // 현재 차선 1/2/3, 모르면 0
    uint8_t  (*get_lane)(void);

    // 전방 거리(m). 없으면 0 (WL-2 distance로 사용)
    uint16_t (*get_front_distance_m)(void);

    // 객체 인식 요약(원인 근거). 지금은 8bit로만 받음. 없으면 0.
    uint8_t  (*get_object_hint)(void);

    // 필요하면 더 추가: 속도, heading 등
} accinfo_provider_t;

// SPI 전송 함수(= spi_interface_module이 제공하는 API를 여기로 주입)
// wl_rpi 쪽으로 “바이트 스트림”을 보내면 됨.
typedef int (*spi_send_fn)(const uint8_t* buf, uint32_t len);

// 설정
typedef struct {
    // WL-2 전송 최소 간격 (ms): 같은 severity 반복 폭주 방지
    uint32_t min_send_interval_ms;

    // 거리 기본값(m): provider가 0이면 이 값으로 대체
    uint16_t default_distance_m;

    // lane 기본값
    uint8_t  default_lane;

    // 로그
    int enable_stdout;
} accsend_config_t;

// 사고 전파 모듈이 외부에서 받는 입력(다른 모듈 -> 사고 전파)
typedef struct {
    uint8_t  severity;     // 1/2/3 (너희 정의)
    acc_type_t type;       // 원인 유형(옵션)
    uint8_t  action;       // [0..15] (옵션: actionTaken 같은 nibble)
    uint16_t distance_m;   // 0이면 provider/front distance 사용
    uint8_t  lane;         // 0이면 provider lane 사용
} acc_event_t;

// 라이프사이클
int  ACCSEND_init(const accsend_config_t* cfg,
                  const accinfo_provider_t* provider,
                  spi_send_fn spi_send);
int  ACCSEND_start(void);
void ACCSEND_stop(void);

// 다른 모듈이 호출: “사고 발생/심각도 갱신” 이벤트
// - 호출은 가볍게(큐에 넣고 return) 처리됨.
int  ACCSEND_on_event(const acc_event_t* ev);

#ifdef __cplusplus
}
#endif
