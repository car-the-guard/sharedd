// src/daemon2.c
#define _GNU_SOURCE
#include <errno.h>
#include <getopt.h>
#include <pthread.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

// ===== 모듈 헤더 포함 =====
#include "accident_announce_module.h"
#include "driving_info_module.h"
#include "bluetooth_module.h"
#include "can_interface_module.h"             // 혹은 can_interface_module.h (파일명 확인 필요)
#include "collision_risk_module.h"
#include "accident_send_module.h"
#include "i2c_interface.h"

// 설정 구조체
typedef struct {
    int enable_stdout;
    int fake_spi;
    int fake_bt;
    int fake_can_tx;
    int fake_i2c;
    const char* can_ifname;
    const char* uart_dev;
    const char* i2c_dev;
    int bt_baud;
} daemon_cfg_t;

static volatile sig_atomic_t g_stop = 0;

static void on_sig(int sig)
{
    (void)sig;
    g_stop = 1;
}

static uint32_t now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(ts.tv_sec * 1000u + ts.tv_nsec / 1000000u);
}

// [Glue Logic]
static void on_can_ultra_cm(uint16_t cm) { DIM_update_ultrasonic(cm * 10); }
static void on_can_speed(uint32_t v) { DIM_update_speed(v); }
static void on_can_accel_x(int16_t raw, int16_t filt) { (void)raw; (void)filt; }
static void on_can_heading(uint16_t deg) { DIM_update_heading(deg); }
static void on_can_vibration(void) { printf("[MAIN] Vibration detected!\n"); }

static void cb_crm_brake_lamp(int level) {
    canif_brake_mode_t mode = BRAKE_OFF;
    if (level == 1) mode = BRAKE_ON;
    else if (level >= 2) mode = BRAKE_BLINK;
    CANIF_send_brakelight(mode, (uint16_t)(now_ms() & 0xFFFF));
    printf("[MAIN] Action: Brake Lamp Level %d\n", level);
}

static void cb_crm_aeb(int enable) {
    uint16_t t = (uint16_t)(now_ms() & 0xFFFF);
    if (enable) CANIF_send_aeb_enable(t);
    else        CANIF_send_aeb_disable(t);
    printf("[MAIN] Action: AEB %d\n", enable);
}

static void cb_crm_notify_scale(int scale) {
    acc_event_t ev = { .severity = (severity_t)scale, .type = ACC_TYPE_HARD_BRAKE };
    ACCSEND_on_event(&ev);
    printf("[MAIN] Action: Notify Scale %d\n", scale);
}

static void cb_crm_airbag(int enable) {
    printf("[MAIN] Action: AIRBAG DEPLOY %d\n", enable);
}

// [Bluetooth Callback]
static void on_bt_cmd(bt_cmd_t cmd) {
    canif_motor4_t m = { .ch = {0,0,0,0} };
    const int8_t SPEED = 60;
    const int8_t TURN  = 50;

    printf("[MAIN] BT Recv: %c\n", (char)cmd);
    switch (cmd) {
    case BT_CMD_D: m.ch[0] = SPEED; break;
    case BT_CMD_S: break;
    case BT_CMD_L: m.ch[2] = TURN;  break;
    case BT_CMD_R: m.ch[3] = TURN;  break;
    default: return;
    }
    CANIF_send_motor4(&m, (uint16_t)(now_ms() & 0xFFFF));
}

// [I2C Callback]
static void on_i2c_packet(const WL3Packet* pkt) {
    // pkt->severity_action 대신 pkt->sev_action 사용
    // 상위 4비트가 Severity
    uint8_t sev = (pkt->sev_action >> 4) & 0x0F;
    
    printf("[DAEMON2] I2C RX: Type=%d, Sev=%d, Lane=%d\n", pkt->accident_type, sev, pkt->lane);

    if (pkt->lane > 0) {
        DIM_update_lane(pkt->lane);
    }

    // 사고 전파 모듈로 전달
    AANN_on_accident(0, pkt->lane, sev);
}

// [Fake SPI]
static pthread_t g_fake_thr;
static int g_fake_run = 0;
static void* fake_spi_thread(void* arg) {
    (void)arg;
    while(g_fake_run && !g_stop) {
        AANN_on_accident(100, 2, 1);
        sleep(3);
    }
    return NULL;
}

static uint8_t pv_get_lane(void) { return DIM_get_lane(); }
static uint16_t pv_get_front_dist_m(void) { return DIM_get_ultrasonic() / 1000; }
static uint8_t pv_get_obj_hint(void) { return 0; }
static int fake_spi_send_bytes(const uint8_t* buf, uint32_t len) {
    (void)buf; (void)len; return 0;
}

int main(int argc, char** argv)
{
    daemon_cfg_t cfg = {
        .enable_stdout = 1,
        .fake_spi = 0,
        .fake_can_tx = 1,
        .fake_i2c = 0,          // [추가] 기본은 Real Mode
        .can_ifname = "can0",
        .uart_dev = "/dev/ttyAMA0",
        .i2c_dev = "/dev/i2c-1", // [추가]
        .bt_baud = 9600,
    };

    static struct option long_opts[] = {
        {"can-if", required_argument, 0, 1},
        {"fake-spi", no_argument, 0, 2},
        {"fake-i2c", no_argument, 0, 3}, // [추가] 3번 ID
        {"uart", required_argument, 0, 4},
        {0,0,0,0}
    };
    int idx, c;
    while((c = getopt_long(argc, argv, "", long_opts, &idx)) != -1) {
        switch(c) {
            case 1: cfg.can_ifname = optarg; break;
            case 2: cfg.fake_spi = 1; break;
            case 3: cfg.fake_i2c = 1; break; // [추가] 옵션 받으면 켜기
            case 4: cfg.uart_dev = optarg; break;
        }
    }

    signal(SIGINT, on_sig);
    signal(SIGTERM, on_sig);

    printf("********************DAEMON STARTED********************\n");
    printf("\n");

    // Init
    DIM_init();

    canif_rx_handlers_t can_rxh = {
        .on_ultrasonic_cm = on_can_ultra_cm, .on_speed_q4 = on_can_speed,
        .on_accel_x = on_can_accel_x, .on_heading_deg = on_can_heading,
        .on_vibration = on_can_vibration
    };
    canif_config_t can_cfg = { .ifname = cfg.can_ifname, .motor_map = {0,1,2,3} };
    if(CANIF_init(&can_cfg, &can_rxh)!=0) fprintf(stderr, "CANIF Init Failed\n");

    crm_callbacks_t crm_cb = {
        .send_brake_lamp_level = cb_crm_brake_lamp, .send_aeb = cb_crm_aeb,
        .notify_accident_scale = cb_crm_notify_scale, .deploy_airbag = cb_crm_airbag
    };
    crm_config_t crm_cfg = { .period_ms=50, .max_data_age_ms=500 };
    CRM_init(&crm_cfg, &crm_cb);

    bt_config_t bt_cfg = { .uart_dev = cfg.uart_dev, .baud = cfg.bt_baud, .enable_stdout = 1 };
    if(BT_init(&bt_cfg, on_bt_cmd)!=0) fprintf(stderr, "BT Init Failed\n");

    i2cif_config_t i2c_cfg = { .devnode = cfg.i2c_dev, .addr = 0x42, .fake_mode = cfg.fake_i2c };
    I2CIF_init(&i2c_cfg, on_i2c_packet);

    aann_config_t aann_cfg = { .min_announce_interval_ms=1000, .dist_bucket_m=5 };
    AANN_init(&aann_cfg);

    // Start
    CANIF_start();
    CRM_start();
    BT_start();
    I2CIF_start();
    AANN_start();

    if(cfg.fake_spi) {
        g_fake_run = 1;
        pthread_create(&g_fake_thr, NULL, fake_spi_thread, NULL);
    }

    // Loop
    uint32_t last_step = 0;
    uint32_t start_time = now_ms(); // 프로그램 시작 시간 기록

    // [상태 플래그] 한 번만 보내기 위해 체크
    int sent_wl4_test = 0;
    int sent_wl3_test = 0;

    while(!g_stop) {
        uint32_t now = now_ms();
        uint32_t elapsed = now - start_time; // 경과 시간 (ms)

        // [시나리오 1] 10초(10000ms) 지남 -> 지자기 정보(WL-4) 전송 (1초마다 반복 전송)
        static uint32_t last_wl4_tx = 0;
        if (elapsed > 10000 && (now - last_wl4_tx >= 5000)) {
            last_wl4_tx = now;
            
            // 임의의 지자기 값 (0~359도 회전 시뮬레이션)
            static uint16_t sim_heading = 0;
            sim_heading = (sim_heading + 10) % 360; 

            // WL-4 전송!
            I2CIF_send_WL4(sim_heading, (uint16_t)now);
            printf("[WL-4] Sent geomagnet info\n");
        }

        // [시나리오 2] 15초(15000ms) 지남 -> 내 사고 정보(WL-3) 1회 전송
        if (elapsed > 15000 && !sent_wl3_test) {
            sent_wl3_test = 1; // 다시 안 보내게 플래그 설정

            WL3Packet pkt = {0};
            pkt.debug_time = (uint16_t)now;
            pkt.accident_type = 1; // 1: 사고 발생
            pkt.lane = 1;          // 1차선
            
            // Severity=2, Action=1 로 설정 (비트 연산)
            // sev_action = (Sev << 4) | Action
            pkt.sev_action = (2 << 4) | 1; 
            
            // Direction 9bit (예: 180도)
            pkt.dir_rsv = (180 & 0x1FF) << 7; 
            
            pkt.accident_id = 0xABCDEF1234567890; // 임의 ID

            // WL-3 전송!
            I2CIF_send_WL3(&pkt);
            printf("[WL-3] Sent Fake Accident\n");
        }

        if((now - last_step) >= 50) {
            last_step = now;
            dim_snapshot_t snap;
            if(DIM_get_snapshot(&snap)==0) {
                crm_inputs_t in;
                in.dist_m = snap.ultra_dist_mm / 1000.0f;
                in.rel_speed_mps = snap.car_speed_1e5_mps / 100000.0f;
                in.rel_accel_mps2 = 0.0f;
                in.object_present = (snap.front_obj.type != DIM_OBJ_NONE);
                in.timestamp_ms = now;
                CRM_update_inputs(&in);
            }
        }

        static uint32_t last_print_ms = 0;
        if (now - last_print_ms >= 15000) {
            last_print_ms = now;
            
            dim_snapshot_t s;
            if (DIM_get_snapshot(&s) == 0) {
                printf("\n******************** [DIM STATUS MONITOR] ********************\n");
                printf(" - Lane: %d (Fresh: %s)\n", 
                       s.lane, (now - s.ts_lane_ms < 2000) ? "YES" : "NO");
                printf(" - Speed: %.2f m/s (Fresh: %s)\n", 
                       (float)s.car_speed_1e5_mps / 100000.0f,
                       (now - s.ts_speed_ms < 2000) ? "YES" : "NO");
                printf(" - Dist: %u mm (Fresh: %s)\n", 
                       s.ultra_dist_mm,
                       (now - s.ts_ultra_ms < 2000) ? "YES" : "NO");
                printf(" - Heading: %u deg (Fresh: %s)\n", 
                       s.heading_deg,
                       (now - s.ts_heading_ms < 2000) ? "YES" : "NO");
                printf(" - Obj: Type=%d, Conf=%u%%, Dist=%u cm\n", 
                       s.front_obj.type, s.front_obj.confidence_pct, s.front_obj.dist_cm);
                printf("\n********************                      ********************\n");
            }
        }

        usleep(5000);
    }

    printf("STOPPING...\n");
    I2CIF_stop();
    BT_stop();
    AANN_stop();
    g_fake_run = 0; 
    if(cfg.fake_spi) pthread_join(g_fake_thr, NULL);
    CRM_stop();
    CANIF_stop();
    DIM_deinit();
    return 0;
}