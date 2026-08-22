#include "vofa.h"

#include "usart.h"
#include "SguanFOC.h"

#include <string.h>


/* =========================================================
 * VOFA+ JustFloat 配置
 * ========================================================= */

/* rad/s -> rpm */
#define RADS_TO_RPM        9.549296586f

/* 发送通道数量 */
#define VOFA_CH_NUM        8

/*
 * 发送周期：
 *
 * 2 ms = 500 Hz
 */
#define VOFA_PERIOD_MS     2U

/*
 * 一帧长度：
 *
 * 8个float：
 * 8 × 4 = 32 Byte
 *
 * JustFloat帧尾：
 * 4 Byte
 *
 * 总计：
 * 36 Byte
 */
#define VOFA_FRAME_SIZE    (VOFA_CH_NUM * sizeof(float) + 4U)


/* =========================================================
 * DMA发送缓冲区
 *
 * 必须使用static。
 *
 * 不能写成VOFA_SendFrame()里的局部数组，
 * 因为DMA启动以后函数已经返回，但DMA仍然需要读取这块内存。
 * ========================================================= */
static uint8_t vofa_tx_buffer[VOFA_FRAME_SIZE];


/*
 * DMA忙标志：
 *
 * 0 = 当前没有发送
 * 1 = DMA正在发送
 */
static volatile uint8_t vofa_tx_busy = 0;
volatile uint32_t g_vofa_tx_complete_count = 0U;
volatile uint32_t g_vofa_tx_error_count = 0U;
volatile uint32_t g_vofa_last_uart_error = HAL_UART_ERROR_NONE;


/* =========================================================
 * VOFA初始化
 * ========================================================= */
void VOFA_Init(void)
{
    vofa_tx_busy = 0;
    g_vofa_tx_complete_count = 0U;
    g_vofa_tx_error_count = 0U;
    g_vofa_last_uart_error = HAL_UART_ERROR_NONE;

    memset(vofa_tx_buffer, 0, sizeof(vofa_tx_buffer));
}


/* =========================================================
 * 准备并启动一帧DMA发送
 * ========================================================= */
static void VOFA_SendFrame(void)
{
    float data[VOFA_CH_NUM];


    /*
     * 如果上一帧还没有发送完，
     * 这一次直接放弃。
     *
     * 绝对不能覆盖正在被DMA读取的缓冲区。
     */
    if (vofa_tx_busy)
    {
        return;
    }


    /* =====================================================
     * CH0：目标机械转速
     *
     * Sguan内部单位：
     * rad/s
     *
     * VOFA显示：
     * rpm
     * ===================================================== */
    data[0] =
        Sguan.foc.Target_Speed * RADS_TO_RPM;


    /* =====================================================
     * CH1：实际机械转速
     *
     * Real_Speed是经过Sguan速度滤波后的速度
     * ===================================================== */
    data[1] =
        Sguan.encoder.Real_Speed * RADS_TO_RPM;


    /* =====================================================
     * CH2：PLL原始速度
     *
     * 用来比较：
     *
     * PLL速度
     *      ↓
     * 速度滤波
     *      ↓
     * Real_Speed
     *
     * 如果高速震动来自编码器速度估计，
     * 这个通道非常有用。
     * ===================================================== */
    data[2] =
        Sguan.encoder.pll.go.OutWe * RADS_TO_RPM;


    /* =====================================================
     * CH3：Iq_ref
     *
     * 在 VelCur_DOUBLE_MODE 中，
     *
     * Velocity.run.Output
     *
     * 就是速度PI输出，
     * 同时直接作为Q轴电流PI参考值。
     * ===================================================== */
    data[3] =
        Sguan.control.Velocity.run.Output;


    /* =====================================================
     * CH4：实际Iq
     * ===================================================== */
    data[4] =
        Sguan.current.Real_Iq;


    /* =====================================================
     * CH5：实际Id
     *
     * SPMSM正常情况下应该尽量接近0A。
     * ===================================================== */
    data[5] =
        Sguan.current.Real_Id;


    /* =====================================================
     * CH6：最终Uq
     *
     * 包含：
     *
     * Q轴PI输出
     * +
     * 反电动势/解耦前馈
     * ===================================================== */
    data[6] =
        Sguan.foc.Uq_in;


    /* =====================================================
     * CH7：Q轴电流PI自身输出
     *
     * 注意和CH6不同：
     *
     * CH7 = PI本身
     *
     * CH6 = PI + 前馈
     *
     * 两者差值能帮助我们观察高速反电动势前馈。
     * ===================================================== */
    data[7] = (float)Sguan.status;


    /* =====================================================
     * float数组复制到DMA发送缓冲区
     * ===================================================== */
    memcpy(
        vofa_tx_buffer,
        data,
        VOFA_CH_NUM * sizeof(float)
    );


    /* =====================================================
     * JustFloat固定帧尾：
     *
     * 00 00 80 7F
     *
     * 对应小端：
     *
     * 0x7F800000
     * ===================================================== */
    vofa_tx_buffer[32] = 0x00;
    vofa_tx_buffer[33] = 0x00;
    vofa_tx_buffer[34] = 0x80;
    vofa_tx_buffer[35] = 0x7F;


    /* DMA开始发送 */
    vofa_tx_busy = 1;


    /*
     * 如果启动DMA失败，
     * 立即解除busy。
     */
    if (HAL_UART_Transmit_DMA(
            &huart2,
            vofa_tx_buffer,
            sizeof(vofa_tx_buffer)
        ) != HAL_OK)
    {
        vofa_tx_busy = 0;
    }
}


/* =========================================================
 * VOFA周期任务
 *
 * 在while(1)里不断调用。
 *
 * 实际每2ms发送一次：
 *
 * 500 Hz
 * ========================================================= */
void VOFA_Task(void)
{
    static uint32_t last_tick = 0;

    uint32_t now = HAL_GetTick();


    if ((uint32_t)(now - last_tick) >= VOFA_PERIOD_MS)
    {
        last_tick = now;

        VOFA_SendFrame();
    }
}


/* =========================================================
 * USART DMA发送完成回调
 *
 * DMA一帧完全发送结束以后，
 * HAL会调用这里。
 * ========================================================= */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        vofa_tx_busy = 0;
        g_vofa_tx_complete_count++;
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        g_vofa_last_uart_error = HAL_UART_GetError(huart);
        g_vofa_tx_error_count++;
        vofa_tx_busy = 0;
    }
}
