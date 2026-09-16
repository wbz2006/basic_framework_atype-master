#ifndef HEAT_CONTROL_H
#define HEAT_CONTROL_H

#include "message_center.h"
#include "stdint.h"

/* 裁判系统热量数据主题，由底盘应用发布，热量控制模块订阅。 */
#define HEAT_CONTROL_DATA_TOPIC "heat_control_data"

/* 2026 规则：17 mm 弹丸每发增加 10 点热量。 */
#define HEAT_CONTROL_DEFAULT_BULLET_HEAT 10.0f
#define HEAT_CONTROL_DEFAULT_MAX_RATE 30.0f
#define HEAT_CONTROL_SETTLE_PERIOD_S 0.1f /* 裁判系统按 10 Hz 结算冷却 */

/**
 * @brief 从裁判系统转发的热量数据
 *
 * current_heat 对应 0x0202 的 shooter_17mm_1_barrel_heat；
 * heat_limit 和 cooling_rate 对应 0x0201 的热量上限、每秒冷却值。
 */
typedef struct
{
    uint16_t current_heat;
    uint16_t heat_limit;
    uint16_t cooling_rate;
    uint8_t valid;
} HeatControlData_s;

/** 热量控制初始化配置。 */
typedef struct
{
    float bullet_heat;
    float max_shoot_rate;
} HeatControl_Init_Config_s;

/** 热量控制实例。 */
typedef struct
{
    float shoot_time;      /* 当前连续发射时长，单位 s */
    float burst_time;      /* 爆发阶段时长，单位 s */
    float shoot_speed;     /* 目标射速，单位 发/s */
    float current_heat;    /* Q1 */
    float heat_limit;      /* Q0 */
    float cooling_rate;    /* 热量/s */
    float bullet_heat;     /* 热量/发 */
    float max_shoot_rate;  /* 物理射速上限，单位 发/s */
    float settle_elapsed;  /* 距离下一次 10 Hz 结算的时间 */
    uint32_t dwt_cnt;      /* 实例独立的 DWT 时间戳 */
    uint8_t data_valid;
    Subscriber_t *data_sub;
} HeatControlInstance;

/**
 * @brief 初始化热量控制实例并订阅裁判热量数据。
 * @param instance 热量控制实例
 * @param config   初始化配置，可传 NULL 使用默认参数
 */
void HeatControlInit(HeatControlInstance *instance, const HeatControl_Init_Config_s *config);

/**
 * @brief 重置爆发阶段和 10 Hz 结算计时。
 * @param instance 热量控制实例
 */
void HeatControlReset(HeatControlInstance *instance);

/**
 * @brief 读取消息中心最新数据并计算目标射速。
 * @param instance 热量控制实例
 * @param firing   当前是否处于连续发射状态
 * @return 目标射速，单位为发/秒
 */
float HeatControlCalculate(HeatControlInstance *instance, uint8_t firing);

#endif
