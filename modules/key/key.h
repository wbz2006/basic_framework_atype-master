#ifndef KEY_H
#define KEY_H

#define KEY_CNT 4

#include "stdint.h"
#include "bsp_gpio.h"

typedef enum
{
    PRESS_OFF = 0,
    PRESS_ON,
} Key_State_e;

typedef enum
{
    KEY_EVENT_NONE = 0,
    KEY_EVENT_SHORT_PRESS,
    KEY_EVENT_LONG_PRESS,
} Key_Event_e;

typedef struct
{
    Key_State_e key_state;
    Key_Event_e key_event;
    uint8_t long_press_flag;
    float press_start_time;
    float press_time;
} KeyPressMonitor_t;

typedef struct
{
    uint8_t key_cnt;
    Key_State_e key_state;
    GPIOInstance *key_gpio_ins;
} KeyInstance;

typedef struct
{
    GPIO_Init_Config_s key_gpio_config;
} Key_Config_s;

KeyInstance *KeyRegister(Key_Config_s *key_config);

void KeyDetectTask();

/**
 * @brief 更新通用按键时长监视器
 *
 * @param key 按键监视器
 * @param is_press 当前按键是否按下
 * @param long_press_time_ms 长按判定时间,单位ms
 * @retval Key_Event_e 本次更新产生的按键事件
 */
Key_Event_e KeyPressMonitorUpdate(KeyPressMonitor_t *key, uint8_t is_press, float long_press_time_ms);

/**
 * @brief 判断按键当前是否处于长按保持状态
 *
 * @param key 按键监视器
 * @retval uint8_t 1:长按保持 0:非长按保持
 */
uint8_t KeyPressMonitorIsLongPress(const KeyPressMonitor_t *key);

/**
 * @brief 复位通用按键时长监视器
 *
 * @param key 按键监视器
 * @retval none
 */
void KeyPressMonitorReset(KeyPressMonitor_t *key);

#endif // !KEY_H
