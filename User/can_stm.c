#include "can_stm.h"

#include <string.h>

#include "comm_mgr_stm.h"
#include "main.h"
#include "motor_can_protocol.h"

#define CAN_STM_LINK_TIMEOUT_MS      (300U)
#define CAN_STM_INIT_RETRY_MS        (500U)
#define CAN_STM_BUS_OFF_RECOVERY_MS  (100U)

static FDCAN_HandleTypeDef s_can;
static bool s_ready;
static uint32_t s_last_command_tick;
static uint32_t s_last_init_tick;
static uint32_t s_last_recovery_tick;
static uint8_t s_last_sequence;
static uint8_t s_last_command;
static bool s_command_rejected;
static uint32_t s_tx_errors;

static bool CAN_STM_LinkActive(void)
{
    return (s_last_command_tick != 0U) &&
           ((HAL_GetTick() - s_last_command_tick) <= CAN_STM_LINK_TIMEOUT_MS);
}

static bool CAN_STM_Send(uint32_t id, const uint8_t data[8])
{
    FDCAN_TxHeaderTypeDef header = {0};
    HAL_StatusTypeDef result;
    if (!s_ready || (HAL_FDCAN_GetTxFifoFreeLevel(&s_can) == 0U))
    {
        ++s_tx_errors;
        return false;
    }
    header.Identifier = id;
    header.IdType = FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = FDCAN_DLC_BYTES_8;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    result = HAL_FDCAN_AddMessageToTxFifoQ(&s_can, &header, data);
    if (result != HAL_OK) ++s_tx_errors;
    return result == HAL_OK;
}

static int32_t CAN_STM_CommandValue(const uint8_t data[8])
{
    switch ((MotorCan_Command_t)data[2])
    {
    case MOTOR_CAN_CMD_SET_MODE:
        return data[3];
    case MOTOR_CAN_CMD_SET_SPEED_RPM:
        return MotorCan_ReadS16(&data[3]);
    case MOTOR_CAN_CMD_SET_POSITION_CDEG:
        return MotorCan_ReadS32(&data[3]);
    default:
        return 0;
    }
}

static void CAN_STM_ProcessRx(void)
{
    while (HAL_FDCAN_GetRxFifoFillLevel(&s_can, FDCAN_RX_FIFO0) > 0U)
    {
        FDCAN_RxHeaderTypeDef header;
        uint8_t data[8];
        if (HAL_FDCAN_GetRxMessage(&s_can, FDCAN_RX_FIFO0, &header, data) != HAL_OK) break;
        if ((header.Identifier != MOTOR_CAN_ID_COMMAND) ||
            (header.IdType != FDCAN_STANDARD_ID) ||
            (header.RxFrameType != FDCAN_DATA_FRAME) ||
            (header.FDFormat != FDCAN_CLASSIC_CAN) ||
            (header.DataLength != FDCAN_DLC_BYTES_8) ||
            (data[0] != MOTOR_CAN_PROTOCOL_VERSION)) continue;

        s_last_command_tick = HAL_GetTick();
        s_last_sequence = data[1];
        s_last_command = data[2];
        if ((MotorCan_Command_t)data[2] == MOTOR_CAN_CMD_PING)
            (void)CommMgr_STM_HandleCommand(data[2], 0);
        else
            s_command_rejected = !CommMgr_STM_HandleCommand(data[2], CAN_STM_CommandValue(data));
    }
}

static bool CAN_STM_ServiceBus(void)
{
    FDCAN_ProtocolStatusTypeDef status = {0};
    const uint32_t now = HAL_GetTick();
    if (HAL_FDCAN_GetProtocolStatus(&s_can, &status) != HAL_OK) return false;
    if (status.BusOff == 0U) return true;
    s_last_command_tick = 0U;
    if ((uint32_t)(now - s_last_recovery_tick) < CAN_STM_BUS_OFF_RECOVERY_MS) return false;
    s_last_recovery_tick = now;
    if ((HAL_FDCAN_Stop(&s_can) != HAL_OK) || (HAL_FDCAN_Start(&s_can) != HAL_OK))
        s_ready = false;
    return false;
}

bool CAN_STM_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    RCC_PeriphCLKInitTypeDef clock = {0};
    FDCAN_FilterTypeDef filter = {0};
    memset(&s_can, 0, sizeof(s_can));
    s_ready = false;
    s_last_init_tick = HAL_GetTick();
    s_last_command_tick = 0U;
    s_last_recovery_tick = 0U;
    s_last_sequence = 0U;
    s_last_command = MOTOR_CAN_CMD_NOP;
    s_command_rejected = false;
    s_tx_errors = 0U;

    clock.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    clock.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&clock) != HAL_OK) return false;
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_FDCAN_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_11;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(GPIOA, &gpio);
    gpio.Pin = GPIO_PIN_9;
    HAL_GPIO_Init(GPIOB, &gpio);

    s_can.Instance = FDCAN1;
    s_can.Init.ClockDivider = FDCAN_CLOCK_DIV1;
    s_can.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
    s_can.Init.Mode = FDCAN_MODE_NORMAL;
    s_can.Init.AutoRetransmission = ENABLE;
    s_can.Init.TransmitPause = DISABLE;
    s_can.Init.ProtocolException = DISABLE;
    /* 170 MHz / 17 / (1 + 15 + 4) = 500 kbit/s, sample point 80%. */
    s_can.Init.NominalPrescaler = 17U;
    s_can.Init.NominalSyncJumpWidth = 4U;
    s_can.Init.NominalTimeSeg1 = 15U;
    s_can.Init.NominalTimeSeg2 = 4U;
    s_can.Init.DataPrescaler = 17U;
    s_can.Init.DataSyncJumpWidth = 4U;
    s_can.Init.DataTimeSeg1 = 15U;
    s_can.Init.DataTimeSeg2 = 4U;
    s_can.Init.StdFiltersNbr = 1U;
    s_can.Init.ExtFiltersNbr = 0U;
    s_can.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
    if (HAL_FDCAN_Init(&s_can) != HAL_OK) return false;

    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0U;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = MOTOR_CAN_ID_COMMAND;
    filter.FilterID2 = 0x7FFU;
    if ((HAL_FDCAN_ConfigFilter(&s_can, &filter) != HAL_OK) ||
        (HAL_FDCAN_ConfigGlobalFilter(&s_can, FDCAN_REJECT, FDCAN_REJECT,
                                     FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK) ||
        (HAL_FDCAN_Start(&s_can) != HAL_OK)) return false;
    s_ready = true;
    return true;
}

void CAN_STM_Task(void)
{
    const uint32_t now = HAL_GetTick();
    if (!s_ready)
    {
        if ((uint32_t)(now - s_last_init_tick) >= CAN_STM_INIT_RETRY_MS)
            (void)CAN_STM_Init();
        return;
    }
    if (CAN_STM_ServiceBus()) CAN_STM_ProcessRx();
}

void CAN_STM_SendState(const MotorMgr_State *state)
{
    uint8_t data[8] = {0};
    uint8_t flags = 0U;
    if ((state == NULL) || !CAN_STM_LinkActive()) return;
    if (state->mode == MOTOR_MGR_MODE_POSITION) flags |= MOTOR_CAN_STATUS_POSITION_MODE;
    if (state->running) flags |= MOTOR_CAN_STATUS_MOTOR_RUNNING;
    if (state->fault) flags |= MOTOR_CAN_STATUS_MOTOR_FAULT;
    flags |= MOTOR_CAN_STATUS_LINK_ACTIVE;
    if (s_command_rejected) flags |= MOTOR_CAN_STATUS_COMMAND_REJECTED;
    data[0] = MOTOR_CAN_PROTOCOL_VERSION;
    data[1] = s_last_sequence;
    data[2] = s_last_command;
    data[3] = flags;
    MotorCan_WriteS16(&data[4], state->speed_rpm);
    MotorCan_WriteU16(&data[6], state->faults);
    (void)CAN_STM_Send(MOTOR_CAN_ID_STATUS, data);

    memset(data, 0, sizeof(data));
    MotorCan_WriteS16(&data[0], state->speed_ref_rpm);
    MotorCan_WriteU16(&data[2], state->position_cdeg);
    MotorCan_WriteU16(&data[4], state->position_ref_cdeg);
    MotorCan_WriteS16(&data[6], state->position_error_cdeg);
    (void)CAN_STM_Send(MOTOR_CAN_ID_REFERENCES, data);

    memset(data, 0, sizeof(data));
    MotorCan_WriteS16(&data[0], state->iq_ma);
    MotorCan_WriteS16(&data[2], state->id_ma);
    MotorCan_WriteS16(&data[4], state->iq_ref_ma);
    MotorCan_WriteS16(&data[6], state->id_ref_ma);
    (void)CAN_STM_Send(MOTOR_CAN_ID_ELECTRICAL, data);
}
