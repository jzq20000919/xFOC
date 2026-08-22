#ifndef __USERDATA_MOTOR_H
#define __USERDATA_MOTOR_H
#include "SguanFOC.h"
/* 电机控制User用户设置·电机参数(SguanFOC用户核心代码) */

// 电机实体参数设置(根据实际需要填写)
static inline void User_MotorSet(void)
{
Sguan.mode = VelCur_DOUBLE_MODE;
    Sguan.flag.PWM_watchdog_limit = 10;

    /* 2. 2804 电机参数 */
    Sguan.identify.Ld   = 0.00086f;
    Sguan.identify.Lq   = 0.00086f;
    Sguan.identify.Ls   = 0.00086f;
    Sguan.identify.Rs   = 2.55f;
    Sguan.identify.Flux = 0.0035f;

    /* 3. 电机与PWM */
    Sguan.motor.Poles = 7;
    Sguan.motor.VBUS  = 12.0f;

    Sguan.motor.Motor_Dir = 1;      // 后面实机确认
    Sguan.motor.PWM_Dir   = 1;      // 我们TIM1使用PWM Mode 1
    Sguan.motor.Duty      = 4249;

    Sguan.motor.Encoder_Dir = 1;   // 后面实机确认

    /* 4. 电流采样 */
    Sguan.motor.Current_Dir0 = -1;   // 后面实机确认
    Sguan.motor.Current_Dir1 = -1;   // 后面实机确认

    Sguan.motor.Current_Num = 1;    // AC两相采样
    Sguan.motor.ADC_Precision = 4096;
    Sguan.motor.Amplifier     = 10.0f;
    Sguan.motor.MCU_Voltage   = 3.3f;
    Sguan.motor.Sampling_Rs   = 0.003f;

    /* 5. 安全参数 */
    Sguan.safe.VBUS_MAX = 16.0f;
    Sguan.safe.VBUS_MIM = 7.4f;
    Sguan.safe.VBUS_watchdog_limit = 1000;

    Sguan.safe.Temp_MAX = 60.0f;
    Sguan.safe.Temp_MIN = -20.0f;
    Sguan.safe.Temp_watchdog_limit = 1000;

    Sguan.safe.Dcur_MAX = 2.0f;
    Sguan.safe.Qcur_MAX = 2.0f;
    Sguan.safe.DQcur_watchdog_limit = 1000;

    Sguan.safe.DISABLED_watchdog_limit = 1000;

    /* 6. 20kHz高速FOC周期 */
    Sguan.PMSM_RUN_T = 0.00005f;
}


#endif // USERDATA_MOTOR_H
