#include "heat_control.h"
#include "bsp_dwt.h"
#include "string.h"

static float HeatControlLimitRate(float rate, float max_rate)
{
    if (!(rate > 0.0f))
        return 0.0f;
    return rate > max_rate ? max_rate : rate;
}

void HeatControlInit(HeatControlInstance *instance, const HeatControl_Init_Config_s *config)
{
    if (instance == NULL)
        return;

    memset(instance, 0, sizeof(*instance));
    instance->bullet_heat = (config != NULL && config->bullet_heat > 0.001f) ?
                                config->bullet_heat : HEAT_CONTROL_DEFAULT_BULLET_HEAT;
    instance->max_shoot_rate = (config != NULL && config->max_shoot_rate > 0.001f) ?
                                    config->max_shoot_rate : HEAT_CONTROL_DEFAULT_MAX_RATE;
    instance->data_sub = SubRegister(HEAT_CONTROL_DATA_TOPIC, sizeof(HeatControlData_s));
    /* 初始化时间基准，避免第一次计算把系统启动以来的时间算入射击时间。 */
    DWT_GetDeltaT(&instance->dwt_cnt);
}

void HeatControlReset(HeatControlInstance *instance)
{
    if (instance == NULL)
        return;

    instance->shoot_time = 0.0f;
    instance->burst_time = 0.0f;
    instance->shoot_speed = 0.0f;
    instance->settle_elapsed = 0.0f;
}

float HeatControlCalculate(HeatControlInstance *instance, uint8_t firing)
{
    HeatControlData_s data;
    float dt;
    uint32_t settle_count = 0;

    if (instance == NULL || instance->data_sub == NULL)
        return 0.0f;

    dt = DWT_GetDeltaT(&instance->dwt_cnt);
    if (!firing)
    {
        HeatControlReset(instance);
        return 0.0f;
    }

    /* 裁判系统按 100 ms 结算一次，200 Hz 任务只在结算边界更新目标射速。 */
    instance->settle_elapsed += dt;
    if (instance->settle_elapsed < HEAT_CONTROL_SETTLE_PERIOD_S)
        return instance->shoot_speed;
    while (instance->settle_elapsed >= HEAT_CONTROL_SETTLE_PERIOD_S)
    {
        instance->settle_elapsed -= HEAT_CONTROL_SETTLE_PERIOD_S;
        settle_count++;
    }

    if (SubGetMessage(instance->data_sub, &data))
    {
        instance->current_heat = (float)data.current_heat;
        instance->heat_limit = (float)data.heat_limit;
        instance->cooling_rate = (float)data.cooling_rate;
        instance->data_valid = data.valid;
    }

    if (!instance->data_valid || instance->heat_limit <= 0.0f || instance->bullet_heat <= 0.001f ||
        instance->current_heat < 0.0f || instance->current_heat >= instance->heat_limit ||
        instance->cooling_rate < 0.0f)
    {
        instance->shoot_speed = 0.0f;
        return 0.0f;
    }

    if (instance->shoot_time == 0.0f)
    {
        float available_heat = instance->heat_limit - instance->current_heat;
        float sustainable_rate = instance->cooling_rate / instance->bullet_heat;

        /* 保留原三阶段策略，但以 Q0-Q1 作为可用热量，避免把当前热量当余量。 */
        /* 结算周期为 0.1 s，爆发时长按热量预算换算为秒。 */
        instance->burst_time = (available_heat + 2.0f * instance->cooling_rate) * 0.1f;
        if (instance->burst_time < HEAT_CONTROL_SETTLE_PERIOD_S)
            instance->burst_time = HEAT_CONTROL_SETTLE_PERIOD_S;

        instance->shoot_speed =
            (10.0f * available_heat - instance->cooling_rate - 3.0f * instance->bullet_heat) /
                (instance->bullet_heat * (instance->burst_time / HEAT_CONTROL_SETTLE_PERIOD_S)) +
            sustainable_rate;
    }
    else if (instance->shoot_time >= instance->burst_time)
    {
        instance->shoot_speed = instance->cooling_rate / instance->bullet_heat;
    }

    instance->shoot_speed = HeatControlLimitRate(instance->shoot_speed, instance->max_shoot_rate);
    instance->shoot_time += settle_count * HEAT_CONTROL_SETTLE_PERIOD_S;
    return instance->shoot_speed;
}
