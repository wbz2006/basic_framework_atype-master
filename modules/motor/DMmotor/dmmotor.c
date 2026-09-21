#include "dmmotor.h"

#include "bsp_log.h"
#include "daemon.h"
#include "general_def.h"
#include "memory.h"
#include "stdlib.h"
#include "string.h"
#include "user_lib.h"

static uint8_t idx;
static DMMotorInstance *dm_motor_instance[DM_MOTOR_CNT];

static uint16_t float_to_uint(
    float x,
    float x_min,
    float x_max,
    uint8_t bits)
{
    const float span = x_max - x_min;
    const float offset = x_min;

    return (uint16_t)((x - offset) *
                      ((float)((1U << bits) - 1U)) /
                      span);
}

static float uint_to_float(
    int x_int,
    float x_min,
    float x_max,
    int bits)
{
    const float span = x_max - x_min;
    const float offset = x_min;

    return ((float)x_int) * span /
               ((float)((1U << bits) - 1U)) +
           offset;
}

static void DMMotorSetMode(DMMotor_Mode_e cmd, DMMotorInstance *motor)
{
    memset(motor->motor_can_instace->tx_buff, 0xff, 7);
    motor->motor_can_instace->tx_buff[7] = (uint8_t)cmd;
    CANTransmit(motor->motor_can_instace, 1);
}

static void DMMotorDecode(CANInstance *motor_can)
{
    uint16_t tmp;
    uint8_t *rxbuff = motor_can->rx_buff;
    DMMotorInstance *motor = (DMMotorInstance *)motor_can->id;
    DM_Motor_Measure_s *measure = &motor->measure;

    DaemonReload(motor->motor_daemon);

    measure->last_position = measure->position;

    tmp = (uint16_t)((rxbuff[1] << 8) | rxbuff[2]);
    measure->position = uint_to_float(tmp, DM_P_MIN, DM_P_MAX, 16);

    tmp = (uint16_t)((rxbuff[3] << 4) | (rxbuff[4] >> 4));
    measure->velocity = uint_to_float(tmp, DM_V_MIN, DM_V_MAX, 12);

    tmp = (uint16_t)(((rxbuff[4] & 0x0f) << 8) | rxbuff[5]);
    measure->torque = uint_to_float(tmp, DM_T_MIN, DM_T_MAX, 12);

    measure->T_Mos = (float)rxbuff[6];
    measure->T_Rotor = (float)rxbuff[7];
}

static void DMMotorLostCallback(void *motor_ptr)
{
    DMMotorInstance *motor = (DMMotorInstance *)motor_ptr;

    /*
     * Keep the callback conservative. The periodic control function will
     * transmit a zero command after the stop flag is set.
     */
    motor->pid_ref = 0.0f;
    motor->stop_flag = MOTOR_STOP;
}

static float DMMotorGetAngleMeasure(
    DMMotorInstance *motor,
    Motor_Control_Setting_s *setting)
{
    if (setting->angle_feedback_source == OTHER_FEED &&
        motor->other_angle_feedback_ptr != NULL)
    {
        return *motor->other_angle_feedback_ptr;
    }

    return motor->measure.position;
}

static float DMMotorGetSpeedMeasure(
    DMMotorInstance *motor,
    Motor_Control_Setting_s *setting)
{
    if (setting->speed_feedback_source == OTHER_FEED &&
        motor->other_speed_feedback_ptr != NULL)
    {
        return *motor->other_speed_feedback_ptr;
    }

    return motor->measure.velocity;
}

static float DMMotorCalculateExternalOutput(DMMotorInstance *motor)
{
    float pid_measure;
    float pid_ref = motor->pid_ref;
    Motor_Control_Setting_s *setting = &motor->motor_settings;

    if (setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
    {
        pid_ref *= -1.0f;
    }

    if ((setting->close_loop_type & ANGLE_LOOP) &&
        setting->outer_loop_type == ANGLE_LOOP)
    {
        pid_measure = DMMotorGetAngleMeasure(motor, setting);
        pid_ref = PIDCalculate(&motor->angle_PID, pid_measure, pid_ref);
    }

    if ((setting->close_loop_type & SPEED_LOOP) &&
        (setting->outer_loop_type & (ANGLE_LOOP | SPEED_LOOP)))
    {
        if ((setting->feedforward_flag & SPEED_FEEDFORWARD) &&
            motor->speed_feedforward_ptr != NULL)
        {
            pid_ref += *motor->speed_feedforward_ptr;
        }

        pid_measure = DMMotorGetSpeedMeasure(motor, setting);
        pid_ref = PIDCalculate(&motor->speed_PID, pid_measure, pid_ref);
    }

    if ((setting->feedforward_flag & CURRENT_FEEDFORWARD) &&
        motor->current_feedforward_ptr != NULL)
    {
        pid_ref += *motor->current_feedforward_ptr;
    }

    if (setting->close_loop_type & CURRENT_LOOP)
    {
        pid_ref = PIDCalculate(
            &motor->current_PID,
            motor->measure.torque,
            pid_ref);
    }

    if (setting->feedback_reverse_flag == FEEDBACK_DIRECTION_REVERSE)
    {
        pid_ref *= -1.0f;
    }

    return pid_ref;
}

static void DMMotorSendMIT(
    DMMotorInstance *motor,
    float position_des,
    float velocity_des,
    float kp,
    float kd,
    float torque_des)
{
    if (motor->stop_flag == MOTOR_STOP)
    {
        position_des = 0.0f;
        velocity_des = 0.0f;
        kp = 0.0f;
        kd = 0.0f;
        torque_des = 0.0f;
    }

    LIMIT_MIN_MAX(position_des, DM_P_MIN, DM_P_MAX);
    LIMIT_MIN_MAX(velocity_des, DM_V_MIN, DM_V_MAX);
    LIMIT_MIN_MAX(kp, DM_KP_MIN, DM_KP_MAX);
    LIMIT_MIN_MAX(kd, DM_KD_MIN, DM_KD_MAX);
    LIMIT_MIN_MAX(torque_des, DM_T_MIN, DM_T_MAX);

    motor->motor_send_mailbox.position_des =
        float_to_uint(position_des, DM_P_MIN, DM_P_MAX, 16);
    motor->motor_send_mailbox.velocity_des =
        float_to_uint(velocity_des, DM_V_MIN, DM_V_MAX, 12);
    motor->motor_send_mailbox.torque_des =
        float_to_uint(torque_des, DM_T_MIN, DM_T_MAX, 12);
    motor->motor_send_mailbox.Kp =
        float_to_uint(kp, DM_KP_MIN, DM_KP_MAX, 12);
    motor->motor_send_mailbox.Kd =
        float_to_uint(kd, DM_KD_MIN, DM_KD_MAX, 12);

    motor->motor_can_instace->tx_buff[0] =
        (uint8_t)(motor->motor_send_mailbox.position_des >> 8);
    motor->motor_can_instace->tx_buff[1] =
        (uint8_t)motor->motor_send_mailbox.position_des;
    motor->motor_can_instace->tx_buff[2] =
        (uint8_t)(motor->motor_send_mailbox.velocity_des >> 4);
    motor->motor_can_instace->tx_buff[3] =
        (uint8_t)(((motor->motor_send_mailbox.velocity_des & 0x0f) << 4) |
                  (motor->motor_send_mailbox.Kp >> 8));
    motor->motor_can_instace->tx_buff[4] =
        (uint8_t)motor->motor_send_mailbox.Kp;
    motor->motor_can_instace->tx_buff[5] =
        (uint8_t)(motor->motor_send_mailbox.Kd >> 4);
    motor->motor_can_instace->tx_buff[6] =
        (uint8_t)(((motor->motor_send_mailbox.Kd & 0x0f) << 4) |
                  (motor->motor_send_mailbox.torque_des >> 8));
    motor->motor_can_instace->tx_buff[7] =
        (uint8_t)motor->motor_send_mailbox.torque_des;

    CANTransmit(motor->motor_can_instace, 1);
}

void DMMotorCaliEncoder(DMMotorInstance *motor)
{
    DMMotorSetMode(DM_CMD_ZERO_POSITION, motor);
    DWT_Delay(0.1);
}

DMMotorInstance *DMMotorInit(Motor_Init_Config_s *config)
{
    DMMotorInstance *motor =
        (DMMotorInstance *)malloc(sizeof(DMMotorInstance));

    if (motor == NULL || config == NULL || idx >= DM_MOTOR_CNT)
    {
        return NULL;
    }

    memset(motor, 0, sizeof(DMMotorInstance));

    motor->motor_settings = config->controller_setting_init_config;
    PIDInit(&motor->current_PID,
            &config->controller_param_init_config.current_PID);
    PIDInit(&motor->speed_PID,
            &config->controller_param_init_config.speed_PID);
    PIDInit(&motor->angle_PID,
            &config->controller_param_init_config.angle_PID);

    motor->other_angle_feedback_ptr =
        config->controller_param_init_config.other_angle_feedback_ptr;
    motor->other_speed_feedback_ptr =
        config->controller_param_init_config.other_speed_feedback_ptr;
    motor->speed_feedforward_ptr =
        config->controller_param_init_config.speed_feedforward_ptr;
    motor->current_feedforward_ptr =
        config->controller_param_init_config.current_feedforward_ptr;

    config->can_init_config.can_module_callback = DMMotorDecode;
    config->can_init_config.id = motor;
    motor->motor_can_instace = CANRegister(&config->can_init_config);

    Daemon_Init_Config_s daemon_config = {
        .callback = DMMotorLostCallback,
        .owner_id = motor,
        .reload_count = 10,
    };
    motor->motor_daemon = DaemonRegister(&daemon_config);

    DMMotorEnable(motor);
    DMMotorSetMode(DM_CMD_MOTOR_MODE, motor);
    DWT_Delay(0.1);
    DMMotorCaliEncoder(motor);
    DWT_Delay(0.1);

    dm_motor_instance[idx++] = motor;
    return motor;
}

void DMMotorSetRef(DMMotorInstance *motor, float ref)
{
    motor->pid_ref = ref;
}

void DMMotorEnable(DMMotorInstance *motor)
{
    motor->stop_flag = MOTOR_ENALBED;
}

void DMMotorStop(DMMotorInstance *motor)
{
    motor->stop_flag = MOTOR_STOP;
}

void DMMotorOuterLoop(DMMotorInstance *motor, Closeloop_Type_e close_loop_type)
{
    motor->motor_settings.outer_loop_type = close_loop_type;
}

void DMMotorChangeFeed(
    DMMotorInstance *motor,
    Closeloop_Type_e loop,
    Feedback_Source_e type)
{
    if (loop == ANGLE_LOOP)
    {
        motor->motor_settings.angle_feedback_source = type;
    }
    else if (loop == SPEED_LOOP)
    {
        motor->motor_settings.speed_feedback_source = type;
    }
    else
    {
        LOGERROR("[dm_motor] loop type error, check memory access and func param");
    }
}

void DMMotorSetMITConfig(
    DMMotorInstance *motor,
    const DMMotor_MIT_Config_s *config)
{
    if (motor == NULL || config == NULL)
    {
        return;
    }

    motor->mit_config = *config;
}

void DMMotorSetMITRef(
    DMMotorInstance *motor,
    float position_des,
    float velocity_des,
    float kp,
    float kd,
    float torque_ff)
{
    const DMMotor_MIT_Config_s config = {
        .position_des = position_des,
        .velocity_des = velocity_des,
        .kp = kp,
        .kd = kd,
        .torque_des = torque_ff,
    };

    DMMotorSetMITConfig(motor, &config);
}

void DMMotorControl(void)
{
    for (size_t i = 0; i < idx; ++i)
    {
        DMMotorInstance *motor = dm_motor_instance[i];
        const float pid_output = DMMotorCalculateExternalOutput(motor);
        const float torque_des = pid_output + motor->mit_config.torque_des;

        /*
         * External impedance mode:
         * the IMU/PID chain owns the control torque, while the DM internal
         * position and velocity gains remain disabled.
         */
        DMMotorSendMIT(motor, 0.0f, 0.0f, 0.0f, 0.0f, torque_des);
    }
}

void DMMotorMITControl(void)
{
    for (size_t i = 0; i < idx; ++i)
    {
        DMMotorInstance *motor = dm_motor_instance[i];

        /*
         * MIT mode:
         * the application supplies p/v/Kp/Kd/tau_ff through
         * DMMotorSetMITConfig(), and this function sends them without
         * calculating the external PID chain.
         */
        DMMotorSendMIT(
            motor,
            motor->mit_config.position_des,
            motor->mit_config.velocity_des,
            motor->mit_config.kp,
            motor->mit_config.kd,
            motor->mit_config.torque_des);
    }
}
