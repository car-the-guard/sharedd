// src/bluetooth_module.c
#define _GNU_SOURCE
#include "bluetooth_module.h"
#include <pthread.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <ctype.h>

// [PART 1] 저수준 구현
static speed_t baud_to_speed(int baud) {
    switch(baud) {
        case 9600: return B9600; case 115200: return B115200; default: return B9600;
    }
}
static int internal_bt_open(const char *dev, int baud) {
    int fd = open(dev, O_RDONLY | O_NOCTTY);
    if(fd < 0) return -1;
    struct termios t;
    if(tcgetattr(fd, &t) < 0) { close(fd); return -1; }
    cfmakeraw(&t);
    cfsetospeed(&t, baud_to_speed(baud));
    cfsetispeed(&t, baud_to_speed(baud));
    t.c_cc[VMIN]=1; t.c_cc[VTIME]=0;
    if(tcsetattr(fd, TCSANOW, &t) < 0) { close(fd); return -1; }
    return fd;
}
static void internal_bt_close(int fd) { if(fd>=0) close(fd); }
static bt_cmd_t internal_bt_read(int fd) {
    unsigned char ch;
    if(read(fd, &ch, 1) <= 0) return BT_CMD_NONE;
    switch(tolower(ch)) {
        case 'd': return BT_CMD_D;
        case 's': return BT_CMD_S;
        case 'l': return BT_CMD_L;
        case 'r': return BT_CMD_R;
    }
    return BT_CMD_NONE;
}

// [PART 2] 모듈 구현
static bt_config_t g_cfg;
static bt_on_cmd_fn g_cb = NULL;
static pthread_t g_thr;
static int g_running = 0;
static int g_fd = -1;

static void* bt_rx_thread(void* arg) {
    (void)arg;
    g_fd = internal_bt_open(g_cfg.uart_dev, g_cfg.baud);
    if(g_fd < 0) { fprintf(stderr, "[BT] Open failed\n"); return NULL; }
    printf("[BT] Started on %s\n", g_cfg.uart_dev);

    while(g_running) {
        bt_cmd_t cmd = internal_bt_read(g_fd);
        if(cmd != BT_CMD_NONE && g_cb) g_cb(cmd);
        if(cmd == BT_CMD_NONE) usleep(10000); // Wait if no data
    }
    internal_bt_close(g_fd);
    return NULL;
}

int BT_init(const bt_config_t* cfg, bt_on_cmd_fn cb) {
    if(!cfg) return -1;
    g_cfg = *cfg; g_cb = cb;
    return 0;
}
int BT_start(void) {
    if(g_running) return 0;
    g_running = 1;
    return pthread_create(&g_thr, NULL, bt_rx_thread, NULL) == 0 ? 0 : -1;
}
void BT_stop(void) {
    if (!g_running) return;
    g_running = 0;

    // [핵심] I2C 때와 마찬가지로, 파일을 먼저 닫아서 read()를 깨워야 합니다.
    // internal_bt_close()나 bt_close()가 fd를 닫는지 확인하세요.
    if (g_fd >= 0) {
        close(g_fd); // <-- 여기서 강제로 닫아야 스레드가 깨어납니다!
        g_fd = -1;
    }

    pthread_join(g_thr, NULL);
}