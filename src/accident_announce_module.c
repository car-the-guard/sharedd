// accident_announce_module.c
#define _GNU_SOURCE
#include "accident_announce_module.h"
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

static pthread_t g_thr;
static pthread_mutex_t g_mtx = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t  g_cv  = PTHREAD_COND_INITIALIZER;

static aann_config_t g_cfg = {
    .min_announce_interval_ms = 1500,
    .dist_bucket_m = 5,
    .enable_stdout = 1,
};

static accident_info_t g_latest;
static uint32_t g_seen_seq = 0;     // consumer가 마지막으로 처리한 seq
static int g_running = 0;

// 중복 안내 방지용 상태
static uint32_t g_last_announce_ms = 0;
static uint16_t g_last_dist_bin = 0;
static lane_t   g_last_lane = LANE_INVALID;
static severity_t g_last_sev = SEV_1;

static uint32_t now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(ts.tv_sec * 1000u + ts.tv_nsec / 1000000u);
}

static const char* lane_str(lane_t lane)
{
    switch (lane) {
    case LANE_1: return "1차선";
    case LANE_2: return "2차선";
    case LANE_3: return "3차선";
    default:     return "차선?";
    }
}

static int is_valid(lane_t lane, severity_t sev)
{
    if (!(lane == LANE_1 || lane == LANE_2 || lane == LANE_3)) return 0;
    if (!(sev == SEV_1 || sev == SEV_2 || sev == SEV_3)) return 0;
    return 1;
}

// 여기 두 개는 나중에 실제 디바이스 출력으로 교체
static void display_show(const accident_info_t* a)
{
    if (!g_cfg.enable_stdout) return;
    printf("[DISPLAY] 외부 사고: 거리=%um, %s, 규모=%u\n",
           (unsigned)a->dist_m, lane_str(a->lane), (unsigned)a->sev);
    fflush(stdout);
}

static void speaker_play(const accident_info_t* a)
{
    if (!g_cfg.enable_stdout) return;
    // TTS 붙이기 전에는 로그로 대체
    printf("[SPEAKER] 전방 %um, %s, 사고 규모 %u. 주의.\n",
           (unsigned)a->dist_m, lane_str(a->lane), (unsigned)a->sev);
    fflush(stdout);
}

static int should_announce(const accident_info_t* a)
{
    // 버킷팅해서 같은 “사고로 간주”하면 재안내 제한
    uint16_t bin = (uint16_t)((a->dist_m / (g_cfg.dist_bucket_m ? g_cfg.dist_bucket_m : 1))
                              * (g_cfg.dist_bucket_m ? g_cfg.dist_bucket_m : 1));

    uint32_t t = now_ms();
    int same_key = (bin == g_last_dist_bin && a->lane == g_last_lane && a->sev == g_last_sev);

    if (same_key) {
        if ((t - g_last_announce_ms) < g_cfg.min_announce_interval_ms) {
            return 0;
        }
    }

    // accept
    g_last_dist_bin = bin;
    g_last_lane = a->lane;
    g_last_sev = a->sev;
    g_last_announce_ms = t;
    return 1;
}

static void* announcer_thread(void* arg)
{
    (void)arg;

    pthread_mutex_lock(&g_mtx);
    while (g_running) {
        // 새 seq가 들어올 때까지 대기
        while (g_running && g_latest.seq == g_seen_seq) {
            pthread_cond_wait(&g_cv, &g_mtx);
        }
        if (!g_running) break;

        // 최신 스냅샷 복사 후 unlock
        accident_info_t a = g_latest;
        g_seen_seq = g_latest.seq;
        pthread_mutex_unlock(&g_mtx);

        if (is_valid(a.lane, a.sev) && should_announce(&a)) {
            display_show(&a);
            speaker_play(&a);
        }

        pthread_mutex_lock(&g_mtx);
    }
    pthread_mutex_unlock(&g_mtx);
    return NULL;
}

int AANN_init(const aann_config_t* cfg)
{
    pthread_mutex_lock(&g_mtx);
    if (cfg) g_cfg = *cfg;
    memset(&g_latest, 0, sizeof(g_latest));
    g_latest.lane = LANE_INVALID;
    g_latest.sev = SEV_1;
    g_latest.seq = 0;
    g_seen_seq = 0;

    g_last_announce_ms = 0;
    g_last_dist_bin = 0;
    g_last_lane = LANE_INVALID;
    g_last_sev = SEV_1;

    pthread_mutex_unlock(&g_mtx);
    return 0;
}

int AANN_start(void)
{
    pthread_mutex_lock(&g_mtx);
    if (g_running) { pthread_mutex_unlock(&g_mtx); return 0; }
    g_running = 1;
    pthread_mutex_unlock(&g_mtx);

    if (pthread_create(&g_thr, NULL, announcer_thread, NULL) != 0) {
        pthread_mutex_lock(&g_mtx);
        g_running = 0;
        pthread_mutex_unlock(&g_mtx);
        return -1;
    }
    return 0;
}

void AANN_stop(void)
{
    pthread_mutex_lock(&g_mtx);
    if (!g_running) { pthread_mutex_unlock(&g_mtx); return; }
    g_running = 0;
    pthread_cond_broadcast(&g_cv);
    pthread_mutex_unlock(&g_mtx);

    pthread_join(g_thr, NULL);
}

// SPI 콜백에서 호출되는 함수: 가볍게. (lock + 저장 + signal)
void AANN_on_accident(uint16_t dist_m, lane_t lane, severity_t sev)
{
    pthread_mutex_lock(&g_mtx);

    g_latest.ts_ms  = now_ms();
    g_latest.dist_m = dist_m;
    g_latest.lane   = lane;
    g_latest.sev    = sev;
    g_latest.seq++;

    pthread_cond_signal(&g_cv);
    pthread_mutex_unlock(&g_mtx);
}

int AANN_get_latest(accident_info_t* out)
{
    if (!out) return -1;
    pthread_mutex_lock(&g_mtx);
    *out = g_latest;
    pthread_mutex_unlock(&g_mtx);
    return 0;
}
