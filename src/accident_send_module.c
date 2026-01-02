// accident_send_module.c
#define _GNU_SOURCE
#include "accident_send_module.h"
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#ifndef ACCSEND_Q_DEPTH
#define ACCSEND_Q_DEPTH 32
#endif

// ---------------- time utils ----------------
static uint32_t now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(ts.tv_sec * 1000u + ts.tv_nsec / 1000000u);
}

// 10ms tick timestamp (wrap OK)
static uint16_t now_10ms(void)
{
    return (uint16_t)((now_ms() / 10u) & 0xFFFFu);
}

// ---------------- module state ----------------
static pthread_t g_thr;
static pthread_mutex_t g_mtx = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t  g_cv  = PTHREAD_COND_INITIALIZER;

static int g_running = 0;

static accsend_config_t g_cfg = {
    .min_send_interval_ms = 200,
    .default_distance_m = 50,
    .default_lane = 2,
    .enable_stdout = 1,
};

static accinfo_provider_t g_pv = {0};
static spi_send_fn g_spi_send = NULL;

// ring queue
typedef struct {
    acc_event_t q[ACCSEND_Q_DEPTH];
    uint32_t head, tail, count;
} ring_t;

static ring_t g_rq = {0};

// rate limit
static uint32_t g_last_send_ms = 0;

// ---------------- helpers ----------------
static uint8_t clamp_u8(uint32_t v, uint8_t lo, uint8_t hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return (uint8_t)v;
}

static uint16_t clamp_u16(uint32_t v, uint16_t lo, uint16_t hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return (uint16_t)v;
}

static uint16_t wl2_pack_dist(uint16_t dist_m, uint8_t rsv4)
{
    uint16_t d12 = (uint16_t)(dist_m & 0x0FFFu);
    return (uint16_t)((d12 << 4) | (rsv4 & 0x0Fu));
}

static uint8_t wl2_pack_sev(uint8_t sev, uint8_t rsv4)
{
    return (uint8_t)(((sev & 0x0Fu) << 4) | (rsv4 & 0x0Fu));
}

static void build_wl2(const acc_event_t* ev, wl2_packet_t* out)
{
    // distance 결정 우선순위: ev.distance_m -> provider.front_distance -> default
    uint16_t dist_m = ev->distance_m;
    if (dist_m == 0 && g_pv.get_front_distance_m) dist_m = g_pv.get_front_distance_m();
    if (dist_m == 0) dist_m = g_cfg.default_distance_m;
    dist_m = clamp_u16(dist_m, 0, 4095);

    // lane 결정: ev.lane -> provider.lane -> default
    uint8_t lane = ev->lane;
    if (lane == 0 && g_pv.get_lane) lane = g_pv.get_lane();
    if (lane == 0) lane = g_cfg.default_lane;
    lane = (lane == 1 || lane == 2 || lane == 3) ? lane : g_cfg.default_lane;

    // severity: 1..15로 clamp (너희는 1..3만 쓰면 됨)
    uint8_t sev = clamp_u8(ev->severity, 1, 15);

    // reserved nibble에 “원인 힌트”를 조금이라도 싣고 싶으면 여기에서 넣어라.
    // 지금은: rsv4 = (type low2bits <<2) | (action low2bits)
    uint8_t rsv_dist = (uint8_t)(((uint8_t)ev->type & 0x3u) << 2) | ((uint8_t)ev->action & 0x3u);
    uint8_t rsv_sev  = 0;
    if (g_pv.get_object_hint) rsv_sev = (uint8_t)(g_pv.get_object_hint() & 0x0Fu);

    out->dist_rsv = wl2_pack_dist(dist_m, rsv_dist);
    out->lane     = lane;
    out->sev_rsv  = wl2_pack_sev(sev, rsv_sev);
    out->ts_10ms  = now_10ms();
}

// ---------------- queue ops ----------------
static int rq_push(const acc_event_t* ev)
{
    if (g_rq.count >= ACCSEND_Q_DEPTH) return 1;
    g_rq.q[g_rq.tail] = *ev;
    g_rq.tail = (g_rq.tail + 1u) % ACCSEND_Q_DEPTH;
    g_rq.count++;
    return 0;
}

static int rq_pop(acc_event_t* out)
{
    if (g_rq.count == 0) return 1;
    *out = g_rq.q[g_rq.head];
    g_rq.head = (g_rq.head + 1u) % ACCSEND_Q_DEPTH;
    g_rq.count--;
    return 0;
}

// ---------------- worker ----------------
static void log_wl2(const wl2_packet_t* p)
{
    if (!g_cfg.enable_stdout) return;
    uint16_t raw = p->dist_rsv;
    uint16_t dist = (raw >> 4) & 0x0FFF;
    uint8_t  rsvd = raw & 0x0F;
    uint8_t  sev  = (p->sev_rsv >> 4) & 0x0F;
    uint8_t  rsvs = p->sev_rsv & 0x0F;

    printf("[ACCSEND] WL-2: dist=%um rsvD=0x%x lane=%u sev=%u rsvS=0x%x ts10=%u\n",
           (unsigned)dist, (unsigned)rsvd, (unsigned)p->lane,
           (unsigned)sev, (unsigned)rsvs, (unsigned)p->ts_10ms);
    fflush(stdout);
}

static void* worker(void* arg)
{
    (void)arg;

    pthread_mutex_lock(&g_mtx);
    while (g_running) {
        while (g_running && g_rq.count == 0) {
            pthread_cond_wait(&g_cv, &g_mtx);
        }
        if (!g_running) break;

        acc_event_t ev;
        if (rq_pop(&ev) != 0) continue;

        // rate limit: 폭주 방지
        uint32_t t = now_ms();
        uint32_t min_iv = g_cfg.min_send_interval_ms;
        if (min_iv && g_last_send_ms && (t - g_last_send_ms) < min_iv) {
            // 너무 빨리 들어오면 최신만 남기고 드랍하고 싶으면 여기 정책을 바꿔라.
            // 지금은 그냥 잠깐 sleep해서 간격 맞춤.
            uint32_t wait_ms = min_iv - (t - g_last_send_ms);
            pthread_mutex_unlock(&g_mtx);
            usleep(wait_ms * 1000u);
            pthread_mutex_lock(&g_mtx);
        }

        // build + send
        wl2_packet_t pkt;
        build_wl2(&ev, &pkt);

        pthread_mutex_unlock(&g_mtx);

        log_wl2(&pkt);

        int rc = -1;
        if (g_spi_send) {
            rc = g_spi_send((const uint8_t*)&pkt, (uint32_t)sizeof(pkt));
        }

        pthread_mutex_lock(&g_mtx);
        g_last_send_ms = now_ms();

        if (g_cfg.enable_stdout) {
            printf("[ACCSEND] spi_send rc=%d\n", rc);
            fflush(stdout);
        }
    }
    pthread_mutex_unlock(&g_mtx);
    return NULL;
}

// ---------------- public api ----------------
int ACCSEND_init(const accsend_config_t* cfg,
                 const accinfo_provider_t* provider,
                 spi_send_fn spi_send)
{
    pthread_mutex_lock(&g_mtx);

    if (cfg) g_cfg = *cfg;
    if (provider) g_pv = *provider;
    g_spi_send = spi_send;

    memset(&g_rq, 0, sizeof(g_rq));
    g_last_send_ms = 0;

    pthread_mutex_unlock(&g_mtx);
    return 0;
}

int ACCSEND_start(void)
{
    pthread_mutex_lock(&g_mtx);
    if (g_running) { pthread_mutex_unlock(&g_mtx); return 0; }
    g_running = 1;
    pthread_mutex_unlock(&g_mtx);

    if (pthread_create(&g_thr, NULL, worker, NULL) != 0) {
        pthread_mutex_lock(&g_mtx);
        g_running = 0;
        pthread_mutex_unlock(&g_mtx);
        return -1;
    }
    return 0;
}

void ACCSEND_stop(void)
{
    pthread_mutex_lock(&g_mtx);
    if (!g_running) { pthread_mutex_unlock(&g_mtx); return; }
    g_running = 0;
    pthread_cond_broadcast(&g_cv);
    pthread_mutex_unlock(&g_mtx);

    pthread_join(g_thr, NULL);
}

int ACCSEND_on_event(const acc_event_t* ev)
{
    if (!ev) return -1;

    pthread_mutex_lock(&g_mtx);
    int full = rq_push(ev);
    if (!full) pthread_cond_signal(&g_cv);
    pthread_mutex_unlock(&g_mtx);

    return full ? 1 : 0;
}

// (테스트용) SPI send stub 예시
// 실제 spi_interface_module이 준비되기 전까지, Yocto에서 stdout으로 확인 가능.

#include "accident_send_module.h"
#include <stdio.h>

static int fake_spi_send(const uint8_t* buf, uint32_t len)
{
    printf("[FAKE_SPI] send %u bytes: ", (unsigned)len);
    for (uint32_t i = 0; i < len; i++) printf("%02X ", buf[i]);
    printf("\n");
    return 0;
}

static uint8_t get_lane_stub(void) { return 2; }
static uint16_t get_dist_stub(void) { return 120; }
static uint8_t get_obj_stub(void) { return 7; }