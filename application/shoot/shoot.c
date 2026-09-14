#include "shoot.h"
#include "robot_def.h"

#include "math.h"
#include "dji_motor.h"
#include "jam_detector.h"
#include "message_center.h"
#include "bsp_dwt.h"
#include "general_def.h"

/* 对于双发射机构的机器人,将下面的数据封装成结构体即可,生成两份shoot应用实例 */
static DJIMotorInstance *friction_l, *friction_r, *loader; // 拨盘电机
// static servo_instance *lid; 需要增加弹舱盖

static Publisher_t *shoot_pub;
static Shoot_Ctrl_Cmd_s shoot_cmd_recv; // 来自cmd的发射控制信息
static Subscriber_t *shoot_sub;
static Shoot_Upload_Data_s shoot_feedback_data; // 来自cmd的发射控制信息
static JamDetectorInstance *loader_jam;

// dwt定时,计算冷却用
static float loader_target_angle = 0;

#define LOADER_ONE_BULLET_MOTOR_ANGLE (ONE_BULLET_DELTA_ANGLE * REDUCTION_RATIO_LOADER)
#define LOADER_ANGLE_TOLERANCE 5.0f

void ShootInit()
{
    // 左摩擦轮
    Motor_Init_Config_s friction_config = {
        .can_init_config = {
            .can_handle = &hcan1,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 20, // 20
                .Ki = 1, // 1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut = 15000,
            },
            .current_PID = {
                .Kp = 0.7, // 0.7
                .Ki = 0.1, // 0.1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut = 15000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
        },
        .motor_type = M3508};
    friction_config.can_init_config.tx_id = 3,
    friction_l = DJIMotorInit(&friction_config);

    friction_config.can_init_config.tx_id = 4; // 右摩擦轮,改txid和方向就行
    friction_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    friction_r = DJIMotorInit(&friction_config);

    // 拨盘电机
    Motor_Init_Config_s loader_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 7,
        },
        .controller_param_init_config = {
            .angle_PID = {
                // 如果启用位置环来控制发弹,需要较大的I值保证输出力矩的线性度否则出现接近拨出的力矩大幅下降
                .Kp = 10, // 10
                .Ki = 0,
                .Kd = 0,
                .MaxOut = 5000,
            },
            .speed_PID = {
                .Kp = 10, // 10
                .Ki = 1, // 1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 5000,
                .MaxOut = 5000,
            },
            .current_PID = {
                .Kp = 0.7, // 0.7
                .Ki = 0.1, // 0.1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 5000,
                .MaxOut = 5000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED, 
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP, // 初始化成SPEED_LOOP,让拨盘停在原地,防止拨盘上电时乱转
            .close_loop_type = CURRENT_LOOP | SPEED_LOOP | ANGLE_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL, // 注意方向设置为拨盘的拨出的击发方向
        },
        .motor_type = M2006 // 英雄使用m3508
    };
    loader = DJIMotorInit(&loader_config);

    JamDetector_Init_Config_s jam_config = {
        .current_threshold = 3000.0f,
        .suspect_timeout_ms = 150.0f,
        .handling_timeout_ms = 800.0f,
        // 回退一发拨盘角度, LOADER_ONE_BULLET_MOTOR_ANGLE 已包含减速比
        .reverse_angle = LOADER_ONE_BULLET_MOTOR_ANGLE,
        .speed_threshold = 200.0f,
        .min_angle_error = 100.0f,
        .min_speed_reference = 100.0f,
    };
    loader_jam = JamDetectorInit(&jam_config, loader);

    shoot_pub = PubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
    shoot_sub = SubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
}

/* 机器人发射机构控制核心任务 */
void ShootTask()
{
    static loader_mode_e last_load_mode = LOAD_STOP;
    static uint8_t jam_handling_active = 0;
    static uint8_t jam_rearm_required = 0;
    uint8_t loader_busy =
        (last_load_mode == LOAD_1_BULLET || last_load_mode == LOAD_3_BULLET) &&
        fabsf(loader->measure.total_angle - loader_target_angle) > LOADER_ANGLE_TOLERANCE;
    uint8_t jam_detection_active;
    JamState_e jam_state = JAM_NORMAL;

    // 从cmd获取控制数据
    SubGetMessage(shoot_sub, &shoot_cmd_recv);
    jam_detection_active =
        loader_busy ||
        (shoot_cmd_recv.load_mode == LOAD_BURSTFIRE) ||
        jam_handling_active;

    // 对shoot mode等于SHOOT_STOP的情况特殊处理,直接停止所有电机(紧急停止)
    if (shoot_cmd_recv.shoot_mode == SHOOT_OFF)
    {
        JamDetectorReset(loader_jam);
        jam_handling_active = 0;
        jam_rearm_required = 0;
        DJIMotorStop(friction_l);
        DJIMotorStop(friction_r);
        DJIMotorStop(loader);
        last_load_mode = LOAD_STOP;
    }
    else // 恢复运行
    {
        DJIMotorEnable(friction_l);
        DJIMotorEnable(friction_r);
        DJIMotorEnable(loader);
    }

    if (shoot_cmd_recv.shoot_mode != SHOOT_OFF &&
        loader_jam != NULL &&
        jam_detection_active)
    {
        jam_state = JamDetectorTask(loader_jam);
        if (jam_state == JAM_CONFIRMED || jam_state == JAM_HANDLING)
        {
            jam_handling_active = 1;
            jam_rearm_required = 1;
        }
        else if (jam_handling_active && jam_state == JAM_NORMAL)
        {
            // 卡弹回退完成,取消本次发射,等待下一次鼠标点击
            jam_handling_active = 0;
            jam_rearm_required = 1;
            last_load_mode = LOAD_STOP;
            loader_target_angle = loader->measure.total_angle;
            JamDetectorReset(loader_jam);
            loader_busy = 0;
        }
    }
    else if (loader_jam != NULL)
    {
        JamDetectorReset(loader_jam);
        jam_handling_active = 0;
    }

    // 卡弹处理完成后,必须先收到一次停止指令,再允许下一次单发或三连发
    if (!jam_handling_active &&
        jam_rearm_required &&
        shoot_cmd_recv.load_mode == LOAD_STOP)
    {
        jam_rearm_required = 0;
    }

    // 如果上一次触发单发或3发指令的时间加上不应期仍然大于当前时间(尚未休眠完毕),直接返回即可
    // 单发模式主要提供给能量机关激活使用(以及英雄的射击大部分处于单发)
    // if (hibernate_time + dead_time > DWT_GetTimeline_ms())
    //     return;

    // 单发或三发执行期间锁存拨盘动作,不受鼠标松开产生的LOAD_STOP影响
    if (shoot_cmd_recv.shoot_mode != SHOOT_OFF &&
        !jam_handling_active &&
        !jam_rearm_required &&
        !(loader_busy &&
          (last_load_mode == LOAD_1_BULLET || last_load_mode == LOAD_3_BULLET)))
    {
        // 若不在休眠状态,根据robotCMD传来的控制模式进行拨盘电机参考值设定和模式切换
        switch (shoot_cmd_recv.load_mode)
        {
        // 停止拨盘
        case LOAD_STOP:
            if (!loader_busy)
            {
                DJIMotorOuterLoop(loader, SPEED_LOOP); // 切换到速度环
                DJIMotorSetRef(loader, 0);             // 同时设定参考值为0,这样停止的速度最快
            }
            break;
        // 单发模式,根据鼠标按下沿触发一次
        case LOAD_1_BULLET:                                                                     // 激活能量机关/干扰对方用,英雄用.
            if (!loader_busy && last_load_mode != LOAD_1_BULLET)
            {
                DJIMotorOuterLoop(loader, ANGLE_LOOP);                                        // 切换到角度环
                loader_target_angle = loader->measure.total_angle + LOADER_ONE_BULLET_MOTOR_ANGLE;
                DJIMotorSetRef(loader, loader_target_angle); // 控制量增加一发弹丸的角度
            }
            break;
        // 三连发,根据鼠标按下沿触发一次
        case LOAD_3_BULLET:
            if (!loader_busy && last_load_mode != LOAD_3_BULLET)
            {
                DJIMotorOuterLoop(loader, ANGLE_LOOP);                                            // 切换到角度环
                loader_target_angle = loader->measure.total_angle + 3 * LOADER_ONE_BULLET_MOTOR_ANGLE;
                DJIMotorSetRef(loader, loader_target_angle); // 增加3发
            }
            break;
        // 连发模式,对速度闭环,射频后续修改为可变,目前固定为1Hz
        case LOAD_BURSTFIRE:
            DJIMotorOuterLoop(loader, SPEED_LOOP);
            DJIMotorSetRef(loader, shoot_cmd_recv.shoot_rate * 360 * REDUCTION_RATIO_LOADER / 8);
            // x颗/秒换算成速度: 已知一圈的载弹量,由此计算出1s需要转的角度,注意换算角速度(DJIMotor的速度单位是angle per second)
            break;
        // 拨盘反转,对速度闭环,后续增加卡弹检测(通过裁判系统剩余热量反馈和电机电流)
        // 也有可能需要从switch-case中独立出来
        case LOAD_REVERSE:
            DJIMotorOuterLoop(loader, SPEED_LOOP);
            // ...
            break;
        default:
            while (1)
                ; // 未知模式,停止运行,检查指针越界,内存溢出等问题
        }
    }
    if (shoot_cmd_recv.shoot_mode != SHOOT_OFF && !loader_busy)
        last_load_mode = shoot_cmd_recv.load_mode;
    else if (shoot_cmd_recv.shoot_mode == SHOOT_OFF)
        last_load_mode = LOAD_STOP;

    // 确定是否开启摩擦轮,后续可能修改为键鼠模式下始终开启摩擦轮(上场时建议一直开启)
    if (shoot_cmd_recv.friction_mode == FRICTION_ON)
    {
        // 根据收到的弹速设置设定摩擦轮电机参考值,需实测后填入
        switch (shoot_cmd_recv.bullet_speed)
        {
        case SMALL_AMU_15:
            DJIMotorSetRef(friction_l, 4000);
            DJIMotorSetRef(friction_r, 4000);
            break;
        case SMALL_AMU_18:
            DJIMotorSetRef(friction_l, 6000);
            DJIMotorSetRef(friction_r, 6000);
            break;
        case SMALL_AMU_30:
            DJIMotorSetRef(friction_l, 8000);
            DJIMotorSetRef(friction_r, 8000);
            break;
        default: // 当前为了调试设定的默认值4000,因为还没有加入裁判系统无法读取弹速.
            DJIMotorSetRef(friction_l, 4000);
            DJIMotorSetRef(friction_r, 4000);
            break;
        }
    }
    else // 关闭摩擦轮
    {
        DJIMotorSetRef(friction_l, 0);
        DJIMotorSetRef(friction_r, 0);
    }

    // 开关弹舱盖
    if (shoot_cmd_recv.lid_mode == LID_CLOSE)
    {
        //...
    }
    else if (shoot_cmd_recv.lid_mode == LID_OPEN)
    {
        //...
    }

    // 反馈数据,目前暂时没有要设定的反馈数据,后续可能增加应用离线监测以及卡弹反馈
    PubPushMessage(shoot_pub, (void *)&shoot_feedback_data);
}
