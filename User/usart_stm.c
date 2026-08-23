#include "usart_stm.h"

#include <string.h>

#include "comm_mgr_stm.h"
#include "main.h"
#include "motor_uart_protocol.h"

#define USART_STM_LINK_TIMEOUT_MS (300U)
#define USART_STM_RX_DMA_SIZE     (128U)

typedef struct
{
    UART_HandleTypeDef uart;
    DMA_HandleTypeDef dma_rx;
    uint8_t dma_buffer[USART_STM_RX_DMA_SIZE];
    uint16_t dma_tail;
    uint8_t parser[MOTOR_UART_MAX_FRAME_SIZE];
    uint8_t parser_length;
    uint8_t telemetry_sequence;
    uint32_t last_command_tick;
    bool ready;
    bool command_rejected;
} USART_STM_Context;

static USART_STM_Context s_usart;

static uint16_t USART_STM_Crc16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFU;
    uint16_t i;
    for (i = 0U; i < length; ++i)
    {
        uint8_t bit;
        crc ^= data[i];
        for (bit = 0U; bit < 8U; ++bit)
            crc = (crc & 1U) ? (uint16_t)((crc >> 1U) ^ 0xA001U) : (uint16_t)(crc >> 1U);
    }
    return crc;
}

static bool USART_STM_LinkActive(void)
{
    return (s_usart.last_command_tick != 0U) &&
           ((HAL_GetTick() - s_usart.last_command_tick) <= USART_STM_LINK_TIMEOUT_MS);
}

static void USART_STM_ParseByte(uint8_t byte)
{
    uint8_t payload_length;
    uint16_t frame_length;
    if (s_usart.parser_length == 0U)
    {
        if (byte == MOTOR_UART_SOF0) s_usart.parser[s_usart.parser_length++] = byte;
        return;
    }
    if (s_usart.parser_length == 1U)
    {
        if (byte == MOTOR_UART_SOF1) s_usart.parser[s_usart.parser_length++] = byte;
        else s_usart.parser_length = (byte == MOTOR_UART_SOF0) ? 1U : 0U;
        return;
    }
    if (s_usart.parser_length >= MOTOR_UART_MAX_FRAME_SIZE)
    {
        s_usart.parser_length = 0U;
        return;
    }
    s_usart.parser[s_usart.parser_length++] = byte;
    if (s_usart.parser_length < 6U) return;
    payload_length = s_usart.parser[5];
    frame_length = (uint16_t)(payload_length + 8U);
    if (payload_length > MOTOR_UART_MAX_PAYLOAD)
    {
        s_usart.parser_length = 0U;
        return;
    }
    if (s_usart.parser_length < frame_length) return;

    if ((MotorUart_ReadU16(&s_usart.parser[6U + payload_length]) ==
         USART_STM_Crc16(&s_usart.parser[2], (uint16_t)(4U + payload_length))) &&
        (s_usart.parser[2] == MOTOR_UART_PROTOCOL_VERSION) &&
        (s_usart.parser[3] == MOTOR_UART_FRAME_COMMAND) &&
        (payload_length == MOTOR_UART_COMMAND_PAYLOAD_SIZE))
    {
        const uint8_t *payload = &s_usart.parser[6];
        const uint8_t command = payload[0];
        const int32_t value = MotorUart_ReadS32(&payload[1]);
        s_usart.last_command_tick = HAL_GetTick();
        if ((MotorUart_Command_t)command == MOTOR_UART_CMD_PING)
            (void)CommMgr_STM_HandleCommand(command, value);
        else
            s_usart.command_rejected = !CommMgr_STM_HandleCommand(command, value);
    }
    s_usart.parser_length = 0U;
}

static void USART_STM_ReadDma(void)
{
    const uint16_t head = (uint16_t)(USART_STM_RX_DMA_SIZE -
                                     __HAL_DMA_GET_COUNTER(&s_usart.dma_rx));
    while (s_usart.dma_tail != head)
    {
        USART_STM_ParseByte(s_usart.dma_buffer[s_usart.dma_tail]);
        if (++s_usart.dma_tail >= USART_STM_RX_DMA_SIZE) s_usart.dma_tail = 0U;
    }
}

bool USART_STM_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    memset(&s_usart, 0, sizeof(s_usart));
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_USART3_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_10 | GPIO_PIN_11;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOC, &gpio);

    s_usart.uart.Instance = USART3;
    s_usart.uart.Init.BaudRate = MOTOR_UART_BAUD_RATE;
    s_usart.uart.Init.WordLength = UART_WORDLENGTH_8B;
    s_usart.uart.Init.StopBits = UART_STOPBITS_1;
    s_usart.uart.Init.Parity = UART_PARITY_NONE;
    s_usart.uart.Init.Mode = UART_MODE_TX_RX;
    s_usart.uart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    s_usart.uart.Init.OverSampling = UART_OVERSAMPLING_16;
    s_usart.uart.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    s_usart.uart.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    s_usart.uart.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if ((HAL_UART_Init(&s_usart.uart) != HAL_OK) ||
        (HAL_UARTEx_SetTxFifoThreshold(&s_usart.uart, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK) ||
        (HAL_UARTEx_SetRxFifoThreshold(&s_usart.uart, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK) ||
        (HAL_UARTEx_DisableFifoMode(&s_usart.uart) != HAL_OK)) return false;

    s_usart.dma_rx.Instance = DMA1_Channel3;
    s_usart.dma_rx.Init.Request = DMA_REQUEST_USART3_RX;
    s_usart.dma_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    s_usart.dma_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    s_usart.dma_rx.Init.MemInc = DMA_MINC_ENABLE;
    s_usart.dma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    s_usart.dma_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    s_usart.dma_rx.Init.Mode = DMA_CIRCULAR;
    s_usart.dma_rx.Init.Priority = DMA_PRIORITY_HIGH;
    if (HAL_DMA_Init(&s_usart.dma_rx) != HAL_OK) return false;
    __HAL_LINKDMA(&s_usart.uart, hdmarx, s_usart.dma_rx);
    if (HAL_UART_Receive_DMA(&s_usart.uart, s_usart.dma_buffer,
                             USART_STM_RX_DMA_SIZE) != HAL_OK) return false;
    s_usart.ready = true;
    return true;
}

void USART_STM_Task(void)
{
    if (s_usart.ready) USART_STM_ReadDma();
}

void USART_STM_SendState(const MotorMgr_State *state)
{
    uint8_t frame[32] = {0};
    uint8_t *payload = &frame[6];
    uint8_t flags = 0U;
    if (!s_usart.ready || (state == NULL)) return;
    if (state->running) flags |= MOTOR_UART_STATUS_MOTOR_RUNNING;
    if (state->fault) flags |= MOTOR_UART_STATUS_MOTOR_FAULT;
    if (s_usart.command_rejected) flags |= MOTOR_UART_STATUS_COMMAND_REJECTED;
    if (USART_STM_LinkActive()) flags |= MOTOR_UART_STATUS_LINK_ACTIVE;
    payload[0] = flags;
    payload[1] = (state->mode == MOTOR_MGR_MODE_POSITION) ?
        MOTOR_UART_MODE_POSITION : MOTOR_UART_MODE_SPEED;
    MotorUart_WriteU16(&payload[2], state->faults);
    MotorUart_WriteS16(&payload[4], state->speed_rpm);
    MotorUart_WriteS16(&payload[6], state->speed_ref_rpm);
    MotorUart_WriteU16(&payload[8], state->position_cdeg);
    MotorUart_WriteU16(&payload[10], state->position_ref_cdeg);
    MotorUart_WriteS16(&payload[12], state->position_error_cdeg);
    MotorUart_WriteS16(&payload[14], state->iq_ma);
    MotorUart_WriteS16(&payload[16], state->id_ma);
    MotorUart_WriteS16(&payload[18], state->iq_ref_ma);
    MotorUart_WriteS16(&payload[20], state->uq_mv);
    MotorUart_WriteS16(&payload[22], state->ud_mv);
    frame[0] = MOTOR_UART_SOF0;
    frame[1] = MOTOR_UART_SOF1;
    frame[2] = MOTOR_UART_PROTOCOL_VERSION;
    frame[3] = MOTOR_UART_FRAME_TELEMETRY;
    frame[4] = s_usart.telemetry_sequence++;
    frame[5] = MOTOR_UART_TELEMETRY_PAYLOAD_SIZE;
    MotorUart_WriteU16(&frame[30], USART_STM_Crc16(&frame[2], 28U));
    (void)HAL_UART_Transmit(&s_usart.uart, frame, sizeof(frame), 2U);
}
