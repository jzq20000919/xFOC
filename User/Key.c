#include "Key.h"

#define RPM_TO_RADS  0.104719755f//转每分钟 -> 弧度每秒
volatile int32_t g_target_rpm = 1000;

volatile float g_target_speed_rad =
        1000.0f * RPM_TO_RADS;

volatile uint8_t g_motor_run = 0;


/* =========================================================
 * 设置目标转速，同时限制在 0~2600 rpm
 * ========================================================= */
static void Key_SetTargetRPM(int32_t rpm)
{
    if (rpm > MOTOR_RPM_MAX)
    {
        rpm = MOTOR_RPM_MAX;
    }

    if (rpm < MOTOR_RPM_MIN)
    {
        rpm = MOTOR_RPM_MIN;
    }

    g_target_rpm = rpm;

    /* rpm -> rad/s */
    g_target_speed_rad =
        (float)rpm * RPM_TO_RADS;
}


/* =========================================================
 * 按键模块初始化
 * ========================================================= */
void Key_ControlInit(void)
{
    /*
     * 上电默认：
     * 电机不运行
     * 目标速度预设为1000 rpm
     */
    g_motor_run = 0;

    Key_SetTargetRPM(0);
}


/* =========================================================
 * 按键扫描
 *
 * KEY1：启停
 * KEY2：+100 rpm
 * KEY3：-100 rpm
 *
 * 你的硬件：
 * 松开 = LOW
 * 按下 = HIGH
 * ========================================================= */
void Key_Task(void)
{
    static uint32_t last_tick = 0;

    static GPIO_PinState key1_last = GPIO_PIN_RESET;
    static GPIO_PinState key2_last = GPIO_PIN_RESET;
    static GPIO_PinState key3_last = GPIO_PIN_RESET;


    /*
     * 每20ms扫描一次
     * 同时完成软件消抖
     */
    if ((HAL_GetTick() - last_tick) < 20)
    {
        return;
    }

    last_tick = HAL_GetTick();


    GPIO_PinState key1 =
        HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin);

    GPIO_PinState key2 =
        HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin);

    GPIO_PinState key3 =
        HAL_GPIO_ReadPin(KEY3_GPIO_Port, KEY3_Pin);


    /* ================= KEY1：启停 ================= */

    /*
     * 上一次 LOW
     * 这一次 HIGH
     *
     * 表示按键刚刚被按下
     */
    if ((key1 == GPIO_PIN_SET) &&
        (key1_last == GPIO_PIN_RESET))
    {
        g_motor_run = !g_motor_run;
    }


    /* ================= KEY2：加速 ================= */

    if ((key2 == GPIO_PIN_SET) &&
        (key2_last == GPIO_PIN_RESET))
    {
        Key_SetTargetRPM(
            g_target_rpm + MOTOR_RPM_STEP
        );
    }


    /* ================= KEY3：减速 ================= */

    if ((key3 == GPIO_PIN_SET) &&
        (key3_last == GPIO_PIN_RESET))
    {
        Key_SetTargetRPM(
            g_target_rpm - MOTOR_RPM_STEP
        );
    }


    /* 保存本次状态，供下一次判断 */
    key1_last = key1;
    key2_last = key2;
    key3_last = key3;
}