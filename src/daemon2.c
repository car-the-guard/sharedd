// main.c (구 daemon2.c) - Yocto Central Controller
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
#include "driving_info_module.h"       // DIM (저장소)
#include "bluetooth_module.h"          // BT
#include "can_interface_module.h"             // CANIF
#include "collision_risk_module.h"     // CRM (충돌 판단)
#include "accident_send_module.h"      // ACCSEND (사고 전파)

// 설정 구조체
typedef struct {
    int enable_stdout;
    int fake_spi;
    int fake_bt;
    int fake_can_tx;
    const char* can_ifname;     // "can0"
    const char* uart_dev;       // "/dev/ttyAMA0"
    int bt_baud;                // 9600
} daemon_cfg_t;

static volatile sig_atomic_t g_stop = 0;

static void on_sig(int sig)
{
    (void)sig;
    g_stop = 1;
}

// 공통 시간 함수 (ms)
static uint32_t now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(ts.tv_sec * 1000u + ts.tv_nsec / 1000000u);
}

// ============================================================
// [Glue Logic] 1. CAN RX -> DIM (데이터 저장소) 연결
// ============================================================
static void on_can_ultra_cm(uint16_t cm) {
    DIM_update_ultrasonic(cm * 10); // cm -> mm 변환 저장
}

static void on_can_speed(uint32_t v) {
    DIM_update_speed(v);
}

static void on_can_accel_x(int16_t raw, int16_t filt) {
    (void)raw; 
    // 가속도 정보 업데이트 함수가 DIM에 있다면 호출 (생략 시 패스)
    // DIM_update_accel(filt); 
}

static void on_can_heading(uint16_t deg) {
    DIM_update_heading(deg);
}

static void on_can_vibration(void) {
    // 충격 감지 시 할 일 (로그 등)
    printf("[MAIN] Vibration detected from CAN!\n");
}

// ============================================================
// [Glue Logic] 2. CRM (판단) -> CAN TX (제어) 연결
// ============================================================
// CRM이 "브레이크 밟아!" 라고 콜백을 부르면, 실제 CAN 메시지로 변환해서 쏘는 역할

static void cb_crm_brake_lamp(int level) {
    canif_brake_mode_t mode = BRAKE_OFF;
    if (level == 1) mode = BRAKE_ON;
    else if (level >= 2) mode = BRAKE_BLINK; // 급제동 시 점멸

    CANIF_send_brakelight(mode, (uint16_t)(now_ms() & 0xFFFF));
    printf("[MAIN] CRM Action: Brake Lamp Level %d\n", level);
}

static void cb_crm_aeb(int enable) {
    uint16_t t = (uint16_t)(now_ms() & 0xFFFF);
    if (enable) CANIF_send_aeb_enable(t);
    else        CANIF_send_aeb_disable(t);
    printf("[MAIN] CRM Action: AEB %s\n", enable ? "ENABLE" : "DISABLE");
}

static void cb_crm_notify_scale(int scale) {
    // 사고 전파 모듈로 이벤트 전달
    acc_event_t ev = {
        .severity = (severity_t)scale, // 타입 매칭 필요
        .type = ACC_TYPE_HARD_BRAKE,   // 예시 타입
        .action = 0,
        .distance_m = 0,
        .lane = 0
    };
    ACCSEND_on_event(&ev);
    printf("[MAIN] CRM Action: Notify Scale %d\n", scale);
}

static void cb_crm_airbag(int enable) {
    // 에어백은 CAN으로 쏘거나, 별도 GPIO 제어
    // 여기선 로그만
    printf("[MAIN] CRM Action: AIRBAG DEPLOY SIGNAL! (enable=%d)\n", enable);
}

// ============================================================
// [Glue Logic] 3. Bluetooth -> Motor Control 연결
// ============================================================
static void on_bt_cmd(bt_cmd_t cmd) 
{
    canif_motor4_t m = { .ch = {0,0,0,0} };
    const int8_t SPEED = 60;
    const int8_t TURN  = 50;

    switch (cmd) {
    case BT_CMD_D: m.ch[0] = SPEED; break; // Forward (예시)
    case BT_CMD_S: break;                  // Stop
    case BT_CMD_L: m.ch[2] = TURN;  break; // Left
    case BT_CMD_R: m.ch[3] = TURN;  break; // Right
    default: return;
    }
    CANIF_send_motor4(&m, (uint16_t)(now_ms() & 0xFFFF));
}

// ============================================================
// [Glue Logic] 4. Accident Send Provider
// ============================================================
static uint8_t pv_get_lane(void) { return DIM_get_lane(); }
static uint16_t pv_get_front_dist_m(void) { return DIM_get_ultrasonic() / 1000; } // mm->m
static uint8_t pv_get_obj_hint(void) { return 0; } // 아직 구현 안됨

static int fake_spi_send_bytes(const uint8_t* buf, uint32_t len) {
    printf("[FAKE_SPI_TX] %u bytes sent\n", len);
    return 0;
}

// ============================================================
// Fake SPI Thread (For Testing)
// ============================================================
static pthread_t g_fake_thr;
static int g_fake_run = 0;
static void* fake_spi_thread(void* arg) {
    (void)arg;
    while(g_fake_run && !g_stop) {
        // 테스트용: 3초마다 사고 정보 수신 시뮬레이션
        AANN_on_accident(100, 2, 1);
        sleep(3);
    }
    return NULL;
}

// ============================================================
// Main
// ============================================================
static void usage(void) {
    printf("usage: main [options]\n --can-if <name>\n --fake-spi\n ...");
}

int main(int argc, char** argv)
{
    daemon_cfg_t cfg = {
        .enable_stdout = 1,
        .fake_spi = 0,
        .fake_can_tx = 1,
        .can_ifname = "can0",
        .uart_dev = "/dev/ttyAMA0",
        .bt_baud = 9600,
    };

    printf("DAEMON STARTED YOCTO LETS GO\n");

    // --- Argument Parsing (생략 가능하지만 유지) ---
    static struct option long_opts[] = {
        {"can-if", required_argument, 0, 1},
        {"fake-spi", no_argument, 0, 2},
        {0,0,0,0}
    };
    int idx;
    while(getopt_long(argc, argv, "", long_opts, &idx) != -1); 
    // (상세 파싱 로직은 위 코드 참조, 여기선 생략)

    // Signals
    signal(SIGINT, on_sig);
    signal(SIGTERM, on_sig);

    printf("[MAIN] System Starting...\n");

    // 1. 모듈 초기화 (Init)
    // ------------------------------------------------
    DIM_init();

    // CAN Interface 설정 & 콜백 연결
    canif_rx_handlers_t can_rxh = {
        .on_ultrasonic_cm = on_can_ultra_cm, // CAN -> DIM
        .on_speed_q4      = on_can_speed,
        .on_accel_x       = on_can_accel_x,
        .on_heading_deg   = on_can_heading,
        .on_vibration     = on_can_vibration
    };
    canif_config_t can_cfg = {
        .ifname = cfg.can_ifname,
        .motor_map = {0,1,2,3}
    };
    if (CANIF_init(&can_cfg, &can_rxh) != 0) {
        fprintf(stderr, "[MAIN] CANIF Init Failed\n");
    }

    // CRM (충돌 판단) 설정 & 액션 콜백 연결
    crm_callbacks_t crm_cb = {
        .send_brake_lamp_level = cb_crm_brake_lamp, // CRM -> CAN TX
        .send_aeb              = cb_crm_aeb,
        .notify_accident_scale = cb_crm_notify_scale,
        .deploy_airbag         = cb_crm_airbag
    };
    crm_config_t crm_cfg = {
        .period_ms = 50,
        .max_data_age_ms = 500 // 0.5초 데이터 없으면 멈춤
        // 나머지는 default 값 사용
    };
    CRM_init(&crm_cfg, &crm_cb);

    // 기타 모듈 Init
    // BT_init(...);
    // AANN_init(...);
    // ACCSEND_init(...);

    // 2. 모듈 시작 (Start Threads)
    // ------------------------------------------------
    CANIF_start();
    CRM_start();
    // BT_start();
    // AANN_start();
    // ACCSEND_start();

    if (cfg.fake_spi) {
        g_fake_run = 1;
        pthread_create(&g_fake_thr, NULL, fake_spi_thread, NULL);
    }

    // 3. Main Loop (Orchestrator)
    // ------------------------------------------------
    // 여기서는 "CRM에 데이터를 넣어주는 역할"과 "상태 모니터링"을 수행
    
    uint32_t last_step_ms = 0;
    const uint32_t STEP_PERIOD = 50; // 50ms 주기

    while (!g_stop) {
        uint32_t now = now_ms();
        
        if ((now - last_step_ms) >= STEP_PERIOD) {
            last_step_ms = now;

            // [Step 1] DIM(저장소)에서 최신 데이터 꺼내오기
            dim_snapshot_t snap;
            if (DIM_get_snapshot(&snap) == 0) {
                // [Step 2] CRM(판단 모듈)에 입력값 밀어넣기
                crm_inputs_t in;
                in.dist_m = snap.ultra_dist_mm / 1000.0f;
                
                // 주의: 여기서 "상대 속도" 계산 로직이 들어가야 함
                // 일단은 내 속도(speed)를 그대로 넣지만, 실제로는 (내속도 - 앞차속도) 필요
                in.rel_speed_mps = (float)snap.car_speed_1e5_mps / 100000.0f; 
                in.rel_accel_mps2 = 0.0f; // 가속도 센서 연동 필요
                
                in.object_present = (snap.front_obj.type != DIM_OBJ_NONE);
                
                // CRM에 업데이트 (이때 CRM 내부에서 timestamp 찍힘)
                CRM_update_inputs(&in);
            }

            // [Step 3] CRM 상태 모니터링 (선택 사항)
            crm_state_t st;
            CRM_get_state(&st);
            
            if (!st.is_data_fresh) {
                // 센서 데이터가 끊김! (로그 1초에 한번만 찍기 등의 처리 필요)
                // printf("[MAIN] Warning: Sensor Data Timeout!\n");
            }
        }

        // CPU 양보
        usleep(5000); // 5ms sleep
    }

    // 4. 종료 (Shutdown)
    // ------------------------------------------------
    printf("[MAIN] System Stopping...\n");
    g_fake_run = 0;
    if (cfg.fake_spi) pthread_join(g_fake_thr, NULL);

    CRM_stop();
    CANIF_stop();
    DIM_deinit();
    // BT_stop(); ...

    return 0;
}