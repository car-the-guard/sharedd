#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    BT_CMD_NONE = 0,
    BT_CMD_D,   // forward
    BT_CMD_S,   // stop
    BT_CMD_L,   // left
    BT_CMD_R,   // right
} bt_cmd_t;

// UART open/close
int  bt_open(const char *dev, int baud);   // baud: 9600, 115200...
void bt_close(int fd);

// blocking read: returns BT_CMD_NONE only if it read junk (keeps running)
bt_cmd_t bt_read_cmd_blocking(int fd, uint8_t *raw_out);

#ifdef __cplusplus
}
#endif
