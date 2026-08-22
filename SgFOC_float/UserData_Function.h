#ifndef __USERDATA_FUNCTION_H
#define __USERDATA_FUNCTION_H
/* 电机控制User用户设置·功能接口 */
/* 用户自己的CODE BEGIN Includes */
// like: #include "main.h"

/* 用户自己的CODE END Includes */
#include "main.h"
#include "tim.h"
#include "adc.h"
#include "opamp.h"
static inline void User_InitialInit(void)
{
    /* 1. 启动三路内部运放 */
    HAL_OPAMP_Start(&hopamp1);
    HAL_OPAMP_Start(&hopamp2);
    HAL_OPAMP_Start(&hopamp3);

    /* 2. ADC自校准 */
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);

    /* 3. 启动编码器 */
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

    /* 4. 先把两个ADC注入组武装好 */
    HAL_ADCEx_InjectedStart(&hadc2);
    HAL_ADCEx_InjectedStart_IT(&hadc1);

    /* 5. 初始PWM占空比清零 */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);

    /* CH4仅负责ADC采样时刻 */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 4248);

    /* 6. 启动CH4，让TIM1开始产生ADC触发 */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    /* 7. 启动三相主PWM */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

    /* 8. 启动三相互补PWM */
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

    /* 9. 启动1kHz低频环 */
    HAL_TIM_Base_Start_IT(&htim2);
}

static inline void User_Delay(unsigned int ms)
{
    HAL_Delay(ms);
}

static inline signed int User_ReadADC_Raw(unsigned char Current_CH)
{
    signed int ADC_num = 0;

    switch (Current_CH)
    {
    case 0:
        /* A/U相：ADC2 Injected Rank1 */
        ADC_num = (signed int)ADC2->JDR1;
        break;

    case 1:
        /* C/W相：ADC1 Injected Rank1 */
        ADC_num = (signed int)ADC1->JDR1;
        break;

    default:
        ADC_num = 0;
        break;
    }

    return ADC_num;
}
static inline float User_Encoder_ReadRad(void)
{
    uint32_t cnt = __HAL_TIM_GET_COUNTER(&htim3);

    return ((float)cnt * 6.283185307179586f / 4096.0f);
}

static inline void User_PwmDuty_Set(
    unsigned short int Duty_u,
    unsigned short int Duty_v,
    unsigned short int Duty_w)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, Duty_u);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, Duty_v);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, Duty_w);
}

static inline float User_VBUS_DataGet(void){
    // float VBUS_num = 0.0f;
    /* Your code for motor VBUS_Voltage Data return if you use it */
    
    // 如果不使用电压功能，返回-9999.0f（正常电压不会是负数）
    return -9999.0f;
}

static inline float User_Temperature_DataGet(void){
    // float Temp_num = 0.0f;
    /* Your code for motor Temperature Data return if you use it */
    
    // 如果不使用温度功能，返回-9999.0f（正常温度不会是这么大的负数）
    return -9999.0f;
}

static inline void User_PWM_OutputDisable(void)
{
    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim1);
}

static inline void User_PWM_OutputEnable(void)
{
    __HAL_TIM_MOE_ENABLE(&htim1);
}

#endif // USERDATA_FUNCTION_H
