// src/bluetooth_module.h
#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 명령어 정의
typedef enum {
    BT_CMD_NONE = 0,
    BT_CMD_D = 'd',  // Forward
    BT_CMD_S = 's',  // Stop
    BT_CMD_L = 'l',  // Left
    BT_CMD_R = 'r'   // Right
} bt_cmd_t;

// 설정 구조체
typedef struct {
    const char* uart_dev; // 예: "/dev/ttyAMA0"
    int baud;             // 예: 9600
    int enable_stdout;
} bt_config_t;

// 콜백 함수 타입
typedef void (*bt_on_cmd_fn)(bt_cmd_t cmd);

// 고수준 API (Main에서 호출)
int  BT_init(const bt_config_t* cfg, bt_on_cmd_fn cb);
int  BT_start(void);
void BT_stop(void);

#ifdef __cplusplus
}
#endif