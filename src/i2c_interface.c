#define _GNU_SOURCE
#include "i2c_interface.h"
#include <pthread.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>

// 패킷 설정
#define WL3_PACKET_LEN 23
#define TX_QUEUE_SIZE  32  // 큐 크기 (패킷 32개 저장 가능)
#define MAX_PKT_SIZE   32  // 최대 패킷 크기 (WL3=23, WL4=4)

// 큐 아이템 구조체
typedef struct {
    uint8_t data[MAX_PKT_SIZE];
    size_t len;
} TxItem;

typedef struct {
    int fd;
    uint8_t addr;
    int fake_mode;
    
    // RX 관련
    pthread_t rx_thr;
    
    // TX 관련 (Queue)
    pthread_t tx_thr;
    TxItem tx_q[TX_QUEUE_SIZE];
    int head, tail, count;
    pthread_mutex_t q_mtx;
    pthread_cond_t  q_cv;

    int running;
    i2c_on_packet_fn cb;
} I2C_Context;

static I2C_Context g_ctx = { 
    .fd = -1, 
    .running = 0,
    .head = 0, .tail = 0, .count = 0,
    .q_mtx = PTHREAD_MUTEX_INITIALIZER,
    .q_cv  = PTHREAD_COND_INITIALIZER
};

// Little Endian Decoders
static uint16_t le16(const uint8_t *p) { 
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8); 
}
static uint64_t le64(const uint8_t *p) {
    return (uint64_t)p[0] | ((uint64_t)p[1] << 8) | ((uint64_t)p[2] << 16) | ((uint64_t)p[3] << 24) |
           ((uint64_t)p[4] << 32) | ((uint64_t)p[5] << 40) | ((uint64_t)p[6] << 48) | ((uint64_t)p[7] << 56);
}

static int read_full(int fd, uint8_t *buf, size_t len) {
    size_t got = 0;
    while (got < len) {
        ssize_t n = read(fd, buf + got, len - got);
        if (n > 0) { got += (size_t)n; continue; }
        if (n == 0) return -1;
        if (errno == EINTR) continue;
        return -1;
    }
    return 0;
}

// [내부 함수] 큐에 패킷 넣기 (Producer)
static int enqueue_packet(const uint8_t* data, size_t len) {
    if (len > MAX_PKT_SIZE) return -1;

    pthread_mutex_lock(&g_ctx.q_mtx);
    
    if (g_ctx.count >= TX_QUEUE_SIZE) {
        // 큐 가득 참: 오래된 것 버리거나, 이번 것 버림 (여기선 이번 것 버림)
        pthread_mutex_unlock(&g_ctx.q_mtx);
        fprintf(stderr, "[I2CIF] TX Queue Full! Packet Dropped.\n");
        return -1;
    }

    // 데이터 복사
    memcpy(g_ctx.tx_q[g_ctx.tail].data, data, len);
    g_ctx.tx_q[g_ctx.tail].len = len;
    
    g_ctx.tail = (g_ctx.tail + 1) % TX_QUEUE_SIZE;
    g_ctx.count++;

    // 대기 중인 TX 스레드 깨움
    pthread_cond_signal(&g_ctx.q_cv);
    pthread_mutex_unlock(&g_ctx.q_mtx);
    return 0;
}

// [스레드] TX 전담 일꾼 (Consumer)
static void* i2c_tx_thread(void* arg) {
    (void)arg;
    while (g_ctx.running) {
        TxItem item;

        // 1. 큐에서 꺼내기 (없으면 대기)
        pthread_mutex_lock(&g_ctx.q_mtx);
        while (g_ctx.count == 0 && g_ctx.running) {
            pthread_cond_wait(&g_ctx.q_cv, &g_ctx.q_mtx);
        }

        if (!g_ctx.running) {
            pthread_mutex_unlock(&g_ctx.q_mtx);
            break;
        }

        // Pop
        item = g_ctx.tx_q[g_ctx.head];
        g_ctx.head = (g_ctx.head + 1) % TX_QUEUE_SIZE;
        g_ctx.count--;
        pthread_mutex_unlock(&g_ctx.q_mtx);

        // 2. 실제 전송 (Blocking 가능)
        if (g_ctx.fd >= 0) {
            if (write(g_ctx.fd, item.data, item.len) != (ssize_t)item.len) {
                perror("[I2CIF] TX Write Error");
            } else {
                // 전송 성공 (너무 빠르면 I2C 버스 점유율 높아지니 살짝 쉼)
                usleep(5000); 
            }
        }
    }
    return NULL;
}

// [스레드] RX 전담 일꾼
static void generate_fake_packet(WL3Packet* pkt) {
    static int tick = 0;
    tick++;
    memset(pkt, 0, sizeof(WL3Packet));
    if (tick < 100) {
        pkt->accident_type = 0; pkt->lane = 2;
    } else if (tick < 200) {
        pkt->accident_type = 1; pkt->lane = 2;
        pkt->sev_action = (2 << 4); pkt->accident_id = 0xDEADBEEF;
    } else {
        pkt->accident_type = 0; if (tick > 250) tick = 0;
    }
}

static void* i2c_rx_thread(void* arg) {
    (void)arg;
    uint8_t buf[WL3_PACKET_LEN];
    WL3Packet pkt;

    // printf("[I2CIF] RX Thread Started. Polling from %02X...\n", g_ctx.addr);
    printf("[I2C] RX Thead Started Receiving from slave %02x \n", g_ctx.addr);

    while (g_ctx.running) {
        if (g_ctx.fake_mode) {
            generate_fake_packet(&pkt);
            if (g_ctx.cb) g_ctx.cb(&pkt);
            usleep(5000 * 1000);
            continue;
        }

        if (g_ctx.fd >= 0) {
            if (read_full(g_ctx.fd, buf, WL3_PACKET_LEN) == 0) {
                pkt.debug_time    = le16(&buf[0]);
                pkt.accident_type = buf[2];
                pkt.sev_action    = buf[3];
                pkt.lane          = buf[4];
                pkt.dir_rsv       = le16(&buf[5]);
                pkt.accident_time = le64(&buf[7]);
                pkt.accident_id   = le64(&buf[15]);
                if (g_ctx.cb) g_ctx.cb(&pkt);
            }
        }
        usleep(5000 * 1000);
    }
    return NULL;
}

// ---------------- Public APIs ----------------

// [수정] 큐에 넣기만 하고 바로 리턴 (Non-blocking)
int I2CIF_send_WL3(const WL3Packet* pkt) {

    // printf("[I2CIF] Enqueue WL-3 >> Type=%u Sev=%u Lane=%u\n",
    //        pkt->accident_type, (pkt->sev_action >> 4), pkt->lane);

    if (g_ctx.fake_mode) return 0; // 가짜 모드는 전송 안 함

    uint8_t buf[23];

    buf[0] = pkt->debug_time & 0xFF;
    buf[1] = (pkt->debug_time >> 8) & 0xFF;
    buf[2] = pkt->accident_type;
    buf[3] = pkt->sev_action;
    buf[4] = pkt->lane;
    buf[5] = pkt->dir_rsv & 0xFF;
    buf[6] = (pkt->dir_rsv >> 8) & 0xFF;
    
    // [Byte 7-14] Accident Time (64bit)
    for(int i=0; i<8; i++) buf[7+i] = (pkt->accident_time >> (i*8)) & 0xFF;
    
    // [Byte 15-22] Accident ID (64bit)
    for(int i=0; i<8; i++) buf[15+i] = (pkt->accident_id >> (i*8)) & 0xFF;

    return enqueue_packet(buf, sizeof(buf));
}

// [수정] 큐에 넣기만 하고 바로 리턴
int I2CIF_send_WL4(uint16_t direction, uint16_t timestamp_ms) {
    // printf("[I2CIF] Enqueue WL-4 >> Dir=%u deg\n", direction);

    if (g_ctx.fake_mode) return 0;

    uint32_t raw = 0;
    raw |= (timestamp_ms & 0xFFFF);
    raw |= ((direction & 0x01FF) << 16);

    uint8_t buf[4];
    buf[0] = raw & 0xFF;
    buf[1] = (raw >> 8) & 0xFF;
    buf[2] = (raw >> 16) & 0xFF;
    buf[3] = (raw >> 24) & 0xFF;

    return enqueue_packet(buf, sizeof(buf));
}

int I2CIF_init(const i2cif_config_t* cfg, i2c_on_packet_fn cb) {
    if (!cfg) return -1;
    g_ctx.cb = cb;
    g_ctx.addr = cfg->addr;
    g_ctx.fake_mode = cfg->fake_mode;

    if (!g_ctx.fake_mode) {
        if (!cfg->devnode) return -1;
        g_ctx.fd = open(cfg->devnode, O_RDWR | O_CLOEXEC);
        if (g_ctx.fd < 0) {
            perror("[I2CIF] Open failed");
            return -1;
        }
        if (ioctl(g_ctx.fd, I2C_SLAVE, g_ctx.addr) < 0) {
            perror("[I2CIF] Ioctl failed");
            close(g_ctx.fd);
            return -1;
        }
    } else {
        g_ctx.fd = -1;
    }
    return 0;
}

int I2CIF_start(void) {
    if (g_ctx.running) return 0;
    g_ctx.running = 1;

    // RX 스레드 시작
    if (pthread_create(&g_ctx.rx_thr, NULL, i2c_rx_thread, NULL) != 0) {
        g_ctx.running = 0;
        return -1;
    }
    
    // TX 스레드 시작
    if (pthread_create(&g_ctx.tx_thr, NULL, i2c_tx_thread, NULL) != 0) {
        g_ctx.running = 0;
        pthread_cancel(g_ctx.rx_thr);
        pthread_join(g_ctx.rx_thr, NULL);
        return -1;
    }
    
    return 0;
}

void I2CIF_stop(void) {
    if (!g_ctx.running) return;
    
    // 1. 플래그 내림
    g_ctx.running = 0;

    // 2. 파일 닫기 (RX 스레드 깨우기)
    if (g_ctx.fd >= 0) {
        close(g_ctx.fd);
        g_ctx.fd = -1;
    }

    // 3. TX 스레드 깨우기 (Cond Signal)
    pthread_mutex_lock(&g_ctx.q_mtx);
    pthread_cond_broadcast(&g_ctx.q_cv);
    pthread_mutex_unlock(&g_ctx.q_mtx);

    // 4. 스레드 합류
    pthread_join(g_ctx.rx_thr, NULL);
    pthread_join(g_ctx.tx_thr, NULL);
}