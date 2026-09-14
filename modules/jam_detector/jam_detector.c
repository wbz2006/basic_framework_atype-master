#include "jam_detector.h"

#include <math.h>
#include <stdlib.h>
#include <string.h> // memset

#include "bsp_dwt.h"  // DWT_GetTimeline_ms
#include "motor_def.h"// ANGLE_LOOP, SPEED_LOOP, Closeloop_Type_e


#define DEFAULT_CURRENT_THRESHOLD   3000.0f  /**< 默认电流阈值, CAN原始值; 调用者应按对应电机的lCur实测调整 */
#define DEFAULT_SUSPECT_TIMEOUT_MS  300.0f   /**< 默认嫌疑超时 300ms */
#define DEFAULT_HANDLING_TIMEOUT_MS 800.0f   /**< 默认处理超时 400ms */
#define DEFAULT_REVERSE_ANGLE       45.0f   /**< 默认回退角度 45° (电机轴) */
#define JAM_ANGLE_TOLERANCE         8.0f    /**< 认为已到达回退目标的角度误差 */
#define JAM_SPEED_TOLERANCE         200.0f  /**< 回退完成时允许的最大电机轴速度 */

/**
 * @brief 初始化卡弹检测器
 *
 * @param config 配置参数, 传入零值自动填充默认值
 * @param motor  拨盘电机实例
 * @return 检测器实例指针
 */
JamDetectorInstance *JamDetectorInit(JamDetector_Init_Config_s *config, DJIMotorInstance *motor)
{
    JamDetectorInstance *jam = (JamDetectorInstance*)malloc(sizeof(JamDetectorInstance));
    memset(jam,0,sizeof(JamDetectorInstance));

    jam->current_threshold   = (config->current_threshold   > 0.0f ) ? config->current_threshold   : DEFAULT_CURRENT_THRESHOLD;
    jam->suspect_timeout_ms  = (config->suspect_timeout_ms  >0.0f  ) ? config->suspect_timeout_ms  : DEFAULT_SUSPECT_TIMEOUT_MS;
    jam->handling_timeout_ms = (config->handling_timeout_ms > 0.0f ) ? config->handling_timeout_ms : DEFAULT_HANDLING_TIMEOUT_MS;
    jam->reverse_angle       = (config->reverse_angle       > 0.0f ) ? config->reverse_angle       : DEFAULT_REVERSE_ANGLE;

    jam->motor = motor;
    jam->state = JAM_NORMAL;

    return jam;

}

/**
 * @brief 核心 FSM: 每周期调用, 返回当前状态
 *
 * 调用者需根据返回值决定是否跳过常规 loader 控制:
 *   JAM_CONFIRMED / JAM_HANDLING → 跳过
 *   JAM_NORMAL / JAM_SUSPECT     → 正常运行
 */
JamState_e JamDetectorTask(JamDetectorInstance *jam)
{
    float now = DWT_GetTimeline_ms();
    float abs_current=fabsf((float)jam->motor->measure.real_current);

    switch (jam->state)
    {
    case JAM_NORMAL:
        if(abs_current>jam->current_threshold)
        {
            jam->state = JAM_SUSPECT;
            jam->suspect_entry_time = now;
        }
        break;

    case JAM_SUSPECT:

        if(abs_current <= jam->current_threshold)
        {
            jam->state=JAM_NORMAL;
        }
        else if ((now-jam->suspect_entry_time) > jam->suspect_timeout_ms)
        {
            jam->state = JAM_CONFIRMED;
        }
        break;

        case JAM_CONFIRMED:

        jam->handling_target_angle = jam->motor->measure.total_angle - jam->reverse_angle;
        DJIMotorOuterLoop(jam->motor, ANGLE_LOOP);
        DJIMotorSetRef(jam->motor, jam->handling_target_angle);

        jam->handling_entry_time = now;
        jam->state = JAM_HANDLING;
        break;

        case JAM_HANDLING:

        DJIMotorOuterLoop(jam->motor, ANGLE_LOOP);
        DJIMotorSetRef(jam->motor, jam->handling_target_angle);
        if ((fabsf(jam->motor->measure.total_angle - jam->handling_target_angle) <= JAM_ANGLE_TOLERANCE &&
             fabsf(jam->motor->measure.speed_aps) <= JAM_SPEED_TOLERANCE) ||
            (now - jam->handling_entry_time) > jam->handling_timeout_ms)
        {
            jam->state = JAM_NORMAL;
        }
        break;

    default:
            jam->state = JAM_NORMAL;
        break;
    }

    return jam->state;
}

/**
 * @brief 强制复位到常规状态
 */
void JamDetectorReset(JamDetectorInstance *jam)
{
    jam->state = JAM_NORMAL;
    jam->handling_target_angle = jam->motor->measure.total_angle;
}
