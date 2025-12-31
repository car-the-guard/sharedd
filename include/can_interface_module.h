// can_interface.h
#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// CAN IDs (11-bit STD)
enum {
    CANID_VIBRATION   = 0x08,
    CANID_AEB_CTRL    = 0x10,
    CANID_MOTOR_CTRL  = 0x12,
    CANID_ULTRASONIC  = 0x24,
    CANID_SPEED       = 0x28,
    CANID_ACCEL       = 0x2C,
    CANID_MAGNETIC    = 0x30,
    CANID_BRAKELIGHT  = 0x48,
};

// 공통 8바이트 페이로드: data32 + time16 + reserved8 + crc8  :contentReference[oaicite:1]{index=1}
typedef struct __attribute__((packed)) {
    uint32_t data;      // byte0~3
    uint16_t time_ms;   // byte4~5 (ms, 2B)
    uint8_t  reserved;  // byte6
    uint8_t  crc8;      // byte7
} canif_payload_t;

// 모터 커맨드: 4바이트에 "채널 4개"를 넣는다.
// - 기본은 [0]=front, [1]=back, [2]=left, [3]=right 같은 식으로 잡고
// - 나중에 order를 바꾸고 싶으면 map만 바꿔서 재배치되게 설계.
typedef struct {
    int8_t ch[4];   // -127~127 (권장). 필요하면 uint8_t로 바꿔도 됨.
} canif_motor4_t;

// 브레이크등 타입 (너 문서 그대로) :contentReference[oaicite:2]{index=2}
typedef enum {
    BRAKE_OFF   = 0,
    BRAKE_ON    = 1,
    BRAKE_BLINK = 2,
    BRAKE_YIELD = 4,
} canif_brake_mode_t;

// RX 콜백들: can_interface가 프레임 파싱해서 이쪽으로 던짐
typedef void (*canif_on_u16_fn)(uint16_t v);
typedef void (*canif_on_i16x2_fn)(int16_t raw, int16_t filt);
typedef void (*canif_on_u32_fn)(uint32_t v);

typedef struct {
    canif_on_u16_fn   on_ultrasonic_cm; // 예: 초음파 거리(cm)로 정하면 여기로
    canif_on_u32_fn   on_speed_q4;      // 속도(0.0001 m/s 단위) 그대로 받기 :contentReference[oaicite:3]{index=3}
    canif_on_i16x2_fn on_accel_x;       // raw/filt (X축) :contentReference[oaicite:4]{index=4}
    canif_on_u16_fn   on_heading_deg;   // 지자기 0~359도 :contentReference[oaicite:5]{index=5}
    void (*on_vibration)(void);         // 충돌/진동 감지 :contentReference[oaicite:6]{index=6}
} canif_rx_handlers_t;

typedef struct {
    const char* ifname;      // "can0"
    uint32_t    rx_poll_us;  // blocking read라 0 권장
    uint8_t     motor_map[4];// motor4.ch[]가 payload.data의 byte0~3에 어떤 순서로 들어갈지
                              // payload_byte[i] = motor.ch[motor_map[i]]
                              // 예: {0,1,2,3}면 그대로
} canif_config_t;

// 라이프사이클
int  CANIF_init(const canif_config_t* cfg, const canif_rx_handlers_t* rxh);
int  CANIF_start(void);
void CANIF_stop(void);

// TX API (yocto -> stm)
int CANIF_send_aeb_enable(uint16_t time_ms);
int CANIF_send_aeb_disable(uint16_t time_ms);

int CANIF_send_brakelight(canif_brake_mode_t mode, uint16_t time_ms);

// “4바이트 모터 커맨드” 송신
int CANIF_send_motor4(const canif_motor4_t* m, uint16_t time_ms);

// 디버그용: 임의 payload 송신
int CANIF_send_raw(uint16_t canid, const canif_payload_t* p);

#ifdef __cplusplus
}
#endif
