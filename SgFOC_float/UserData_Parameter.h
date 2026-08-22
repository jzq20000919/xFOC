#ifndef __USERDATA_PARAMETER_H
#define __USERDATA_PARAMETER_H
#include "SguanFOC.h"
/* 电机控制User用户设置·BPF和PID和PLL运行参数 */

static inline void User_ParameterSet(void)
{
    /* 电流反馈滤波 */
    Sguan.bpf.CurrentD.Wc = 12566.37f;
    Sguan.bpf.CurrentQ.Wc = 12566.37f;
    Sguan.bpf.Encoder.Wc  = 300.0f;
/* Id 电流环 */
    Sguan.control.Current_D.Wc = 100.0f;
    Sguan.control.Current_D.Kp = 2.70f;
    Sguan.control.Current_D.Ki = 8011.0f;
    Sguan.control.Current_D.Kd = 0.0f;
    Sguan.control.Current_D.OutMax = 12.0f;
    Sguan.control.Current_D.OutMin = -12.0f;
    Sguan.control.Current_D.IntMax = 12.0f;
    Sguan.control.Current_D.IntMin = -12.0f;
/* Iq 电流环 */
    Sguan.control.Current_Q.Wc = 100.0f;
    Sguan.control.Current_Q.Kp = 2.70f;
    Sguan.control.Current_Q.Ki = 8011.0f;
    Sguan.control.Current_Q.Kd = 0.0f;
    Sguan.control.Current_Q.OutMax = 12.0f;
    Sguan.control.Current_Q.OutMin = -12.0f;
    Sguan.control.Current_Q.IntMax = 12.0f;
    Sguan.control.Current_Q.IntMin = -12.0f;
#if Open_PI_Control

    /* ================= 速度PI ================= */

    Sguan.control.Velocity.Wc = 100.0f;
    Sguan.control.Velocity.Kp = 0.00668f;
    Sguan.control.Velocity.Ki = 0.01630f;
    Sguan.control.Velocity.Kd = 0.0f;

    /* 速度环输出就是 Iq_ref，因此按照2A限幅 */
    Sguan.control.Velocity.OutMax = 2.0f;
    Sguan.control.Velocity.OutMin = -2.0f;

    Sguan.control.Velocity.IntMax = 2.0f;
    Sguan.control.Velocity.IntMin = -2.0f;

#endif

    /* 位置环暂时不用，保持原值即可 */
    Sguan.control.Position.Wc = 100.0f;
    Sguan.control.Position.Kp = 8.0f;
    Sguan.control.Position.Ki = 0.0f;
    Sguan.control.Position.Kd = 0.0f;

    Sguan.control.Position.OutMax = 210.0f;
    Sguan.control.Position.OutMin = -210.0f;

    Sguan.control.Position.IntMax = 150.0f;
    Sguan.control.Position.IntMin = -150.0f;


    /*
     * 关键：
     * 高频环 = 20kHz
     * Response = 20
     * => 速度PI = 20k / 20 = 1kHz
     */
    Sguan.control.Response = 20;


    /* 编码器PLL先保持Sguan原参数 */
    Sguan.encoder.pll.Kp = 650.0f;
    Sguan.encoder.pll.Ki = 210000.0f;
}

#endif // USERDATA_PARAMETER_H
