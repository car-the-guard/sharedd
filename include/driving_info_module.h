// driving_info_module.h
#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 차선: 3차선 기준
typedef enum {
    DIM_LANE_UNKNOWN = 0,
    DIM_LANE_1 = 1,
    DIM_LANE_2 = 2,
    DIM_LANE_3 = 3
} dim_lane_t;

// 전방 객체 종류(필요시 확장)
typedef enum {
    DIM_OBJ_NONE = 0,
    DIM_OBJ_CAR,
    DIM_OBJ_PERSON,
    DIM_OBJ_CONE,
    DIM_OBJ_UNKNOWN
} dim_obj_type_t;

// 전방 객체 1개(“가장 위험/가장 가까운” 하나만 저장하는 최소 설계)
typedef struct {
    dim_obj_type_t type;
    uint8_t  confidence_pct;  // 0~100
    int16_t  rel_lane;        // -1:좌, 0:내차선, +1:우 (AI가 주는 기준으로)
    uint16_t dist_cm;         // 객체까지 거리(cm) (AI 추정)
} dim_front_object_t;

// 모듈이 저장하는 전체 스냅샷
typedef struct {
    // 값
    dim_lane_t lane;              // 현재 주행 차선(1/2/3)
    dim_front_object_t front_obj; // 전방 객체
    uint32_t car_speed_1e5_mps;   // 차량 속도 (0.00001 m/s 단위)
    uint16_t ultra_dist_mm;       // 초음파 전방 거리(mm)
    uint16_t heading_deg;         // 지자기 방향(0~359)

    // 메타(신선도/검증용)
    uint32_t ts_lane_ms;
    uint32_t ts_obj_ms;
    uint32_t ts_speed_ms;
    uint32_t ts_ultra_ms;
    uint32_t ts_heading_ms;

    uint32_t seq;                 // 업데이트 카운터(전체에 대해 증가)
} dim_snapshot_t;

// init/deinit
int  DIM_init(void);
void DIM_deinit(void);

// 업데이트(“수신자” 모듈들이 호출)
void DIM_update_lane(dim_lane_t lane);
void DIM_update_front_object(const dim_front_object_t* obj);
void DIM_update_speed(uint32_t speed_1e5_mps);
void DIM_update_ultrasonic(uint16_t dist_mm);
void DIM_update_heading(uint16_t heading_deg); // 0~359로 정규화해 저장

// 조회(“사용자” 모듈들이 호출)
int  DIM_get_snapshot(dim_snapshot_t* out); // 원자적 스냅샷 복사
dim_lane_t DIM_get_lane(void);
uint32_t   DIM_get_speed(void);
uint16_t   DIM_get_ultrasonic(void);
uint16_t   DIM_get_heading(void);
int        DIM_get_front_object(dim_front_object_t* out);

// 신선도 체크(모듈 내부 타임스탬프 기준)
int DIM_is_fresh_lane(uint32_t max_age_ms);
int DIM_is_fresh_speed(uint32_t max_age_ms);
int DIM_is_fresh_ultra(uint32_t max_age_ms);
int DIM_is_fresh_heading(uint32_t max_age_ms);
int DIM_is_fresh_obj(uint32_t max_age_ms);

#ifdef __cplusplus
}
#endif
