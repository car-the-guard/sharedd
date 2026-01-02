// src/i2c_interface.h
#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// [수정] 팀원의 proto_wl3.h 와 100% 동일하게 맞춤 (총 23바이트)
#pragma pack(push, 1)
typedef struct {
    uint16_t debug_time;      // [0-1]
    uint8_t  accident_type;   // [2]
    uint8_t  sev_action;      // [3] 상위4bit:Severity, 하위4bit:Action
    uint8_t  lane;            // [4]
    uint16_t dir_rsv;         // [5-6] Direction(9bit) + Reserved
    uint64_t accident_time;   // [7-14]
    uint64_t accident_id;     // [15-22]
} WL3Packet;
#pragma pack(pop)

typedef void (*i2c_on_packet_fn)(const WL3Packet* pkt);

typedef struct {
    const char* devnode;
    uint8_t     addr;
    int         fake_mode; // 가짜 모드 지원
} i2cif_config_t;

int  I2CIF_init(const i2cif_config_t* cfg, i2c_on_packet_fn cb);
int  I2CIF_start(void);
void I2CIF_stop(void);
int I2CIF_send_WL3(const WL3Packet* pkt);
int I2CIF_send_WL4(uint16_t direction, uint16_t timestamp_ms);

#ifdef __cplusplus
}
#endif