#ifndef DMMOTOR_H
#define DMMOTOR_H

#include <stdint.h>

#include "bsp_can.h"
#include "controller.h"
#include "motor_def.h"
#include "daemon.h"

#define DM_MOTOR_CNT 4

/* DM4310 protocol limits */
#define DM_P_MIN (-12.5f)
#define DM_P_MAX (12.5f)
#define DM_V_MIN (-20.94f)
#define DM_V_MAX (20.94f)
#define DM_T_MIN (-12.5f)
#define DM_T_MAX (12.5f)
#define DM_KP_MIN (0.0f)
#define DM_KP_MAX (500.0f)
#define DM_KD_MIN (0.0f)
#define DM_KD_MAX (5.0f)

typedef struct
{
    uint8_t id;
    uint8_t state;
    float velocity;
    float last_position;
    float position;
    float torque;
    float T_Mos;
    float T_Rotor;
    int32_t total_round;
} DM_Motor_Measure_s;

/* Encoded MIT command fields used by the CAN packet. */
typedef struct
{
    uint16_t position_des;
    uint16_t velocity_des;
    uint16_t torque_des;
    uint16_t Kp;
    uint16_t Kd;
} DMMotor_Send_s;

/* Physical-unit MIT command configuration. */
typedef struct
{
    float position_des; /* rad */
    float velocity_des; /* rad/s */
    float torque_des;   /* N*m */
    float kp;
    float kd;
} DMMotor_MIT_Config_s;

typedef struct
{
    DM_Motor_Measure_s measure;
    Motor_Control_Setting_s motor_settings;

    PIDInstance current_PID;
    PIDInstance speed_PID;
    PIDInstance angle_PID;

    float *other_angle_feedback_ptr;
    float *other_speed_feedback_ptr;
    float *speed_feedforward_ptr;
    float *current_feedforward_ptr;
    float pid_ref;

    /* Used by DMMotorMITControl(). */
    DMMotor_MIT_Config_s mit_config;
    DMMotor_Send_s motor_send_mailbox;

    Motor_Working_Type_e stop_flag;
    CANInstance *motor_can_instace;
    DaemonInstance *motor_daemon;
    uint32_t lost_cnt;
} DMMotorInstance;

typedef enum
{
    DM_CMD_MOTOR_MODE = 0xfc,   /* Enable and respond to commands. */
    DM_CMD_RESET_MODE = 0xfd,   /* Stop. */
    DM_CMD_ZERO_POSITION = 0xfe, /* Set current position as encoder zero. */
    DM_CMD_CLEAR_ERROR = 0xfb   /* Clear motor over-temperature error. */
} DMMotor_Mode_e;

DMMotorInstance *DMMotorInit(Motor_Init_Config_s *config);

void DMMotorSetRef(DMMotorInstance *motor, float ref);
void DMMotorOuterLoop(DMMotorInstance *motor, Closeloop_Type_e close_loop_type);
void DMMotorEnable(DMMotorInstance *motor);
void DMMotorStop(DMMotorInstance *motor);
void DMMotorCaliEncoder(DMMotorInstance *motor);

/**
 * @brief Change the feedback source used by the external PID loops.
 */
void DMMotorChangeFeed(
    DMMotorInstance *motor,
    Closeloop_Type_e loop,
    Feedback_Source_e type);

/**
 * @brief Configure the physical-unit MIT command.
 *
 * This command is used by DMMotorMITControl(). In DMMotorControl(), only
 * torque_des is used as external torque feedforward; p/v/Kp/Kd are zeroed.
 */
void DMMotorSetMITConfig(
    DMMotorInstance *motor,
    const DMMotor_MIT_Config_s *config);

/**
 * @brief Set the physical-unit MIT command.
 */
void DMMotorSetMITRef(
    DMMotorInstance *motor,
    float position_des,
    float velocity_des,
    float kp,
    float kd,
    float torque_ff);

/**
 * @brief IMU external impedance control.
 *
 * The external angle/speed/current PID chain is calculated in this function.
 * The transmitted MIT fields are p=0, v=0, Kp=0, Kd=0 and torque=PID output
 * plus torque feedforward.
 */
void DMMotorControl(void);

/**
 * @brief IMU outer loop plus DM internal MIT position/speed loop control.
 *
 * The MIT command configured by DMMotorSetMITConfig() is sent directly. The
 * external PID chain is not calculated in this function.
 */
void DMMotorMITControl(void);

#endif /* DMMOTOR_H */
