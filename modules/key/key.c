#include "key.h"
#include "string.h"
#include "stdlib.h"
#include "bsp_dwt.h"

static KeyInstance *key_instance[KEY_CNT] = {NULL};
static uint8_t idx = 0;

/**
 * @brief 更新通用按键时长监视器
 *
 * @param key 按键监视器
 * @param is_press 当前按键是否按下
 * @param long_press_time_ms 长按判定时间,单位ms
 * @retval Key_Event_e 本次更新产生的按键事件
 */
Key_Event_e KeyPressMonitorUpdate(KeyPressMonitor_t *key, uint8_t is_press, float long_press_time_ms)
{
    float now;

    if (key == NULL)
        return KEY_EVENT_NONE;

    now = DWT_GetTimeline_ms();
    key->key_event = KEY_EVENT_NONE;

    if (is_press)
    {
        if (key->key_state == PRESS_OFF)
        {
            key->key_state = PRESS_ON;
            key->press_start_time = now;
            key->press_time = 0.0f;
            key->long_press_flag = 0;
        }
        else
        {
            key->press_time = now - key->press_start_time;
            if ((key->long_press_flag == 0) && (key->press_time >= long_press_time_ms))
            {
                key->long_press_flag = 1;
                key->key_event = KEY_EVENT_LONG_PRESS;
            }
        }
    }
    else
    {
        if (key->key_state == PRESS_ON)
        {
            key->press_time = now - key->press_start_time;
            if (key->long_press_flag == 0)
                key->key_event = KEY_EVENT_SHORT_PRESS;
        }
        key->key_state = PRESS_OFF;
        key->long_press_flag = 0;
    }

    return key->key_event;
}

/**
 * @brief 判断按键当前是否处于长按保持状态
 *
 * @param key 按键监视器
 * @retval uint8_t 1:长按保持 0:非长按保持
 */
uint8_t KeyPressMonitorIsLongPress(const KeyPressMonitor_t *key)
{
    if (key == NULL)
        return 0;

    return (key->key_state == PRESS_ON) && (key->long_press_flag != 0);
}

/**
 * @brief 复位通用按键时长监视器
 *
 * @param key 按键监视器
 * @retval none
 */
void KeyPressMonitorReset(KeyPressMonitor_t *key)
{
    if (key != NULL)
        memset(key, 0, sizeof(KeyPressMonitor_t));
}

// 按键检测任务
void KeyDetectTask()
{
    KeyInstance *key;

    for(int i = 0; i < KEY_CNT; i++)
    {
        key = key_instance[i];

        if(key == NULL)
            return;

        if(GPIORead(key->key_gpio_ins))
        {
            DWT_Delay_ms(100);
            if(GPIORead(key->key_gpio_ins))
            {
                key->key_state = PRESS_ON;
                continue;
            }
        }
        key->key_state = PRESS_OFF;
    }
}

// 按键中断回调
static void KeyEXITCallback(GPIOInstance *_instance)
{
    KeyInstance *ins = (KeyInstance*)_instance->id;

    KeyDetectTask();
    if(ins->key_state == PRESS_ON)
        ins->key_cnt++;
}

// 按键注册函数
KeyInstance *KeyRegister(Key_Config_s *key_config)
{
    if(idx >= KEY_CNT)    // 超过最大实例数
        while(1)
        ;

    KeyInstance *instance = (KeyInstance*)malloc(sizeof(KeyInstance));
    memset(instance, 0, sizeof(KeyInstance));

    instance->key_cnt = 0;  // 按键计数初始化为0
    instance->key_state = PRESS_OFF;

    key_config->key_gpio_config.gpio_model_callback = KeyEXITCallback;
    key_config->key_gpio_config.id = instance;
    instance->key_gpio_ins = GPIORegister(&key_config->key_gpio_config);

    key_instance[idx++] = instance;
    return instance;
}
