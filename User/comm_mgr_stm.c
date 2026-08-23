#include "comm_mgr_stm.h"

#include "can_stm.h"
#include "motor_mgr.h"
#include "usart_stm.h"
#include "main.h"

#define COMM_MGR_STM_TELEMETRY_MS (20U)

void CommMgr_STM_Init(void)
{
    (void)CAN_STM_Init();
    (void)USART_STM_Init();
}

bool CommMgr_STM_HandleCommand(uint8_t command, int32_t value)
{
    switch ((CommMgr_STM_Command)command)
    {
    case COMM_MGR_STM_CMD_NOP:
    case COMM_MGR_STM_CMD_PING:
        return true;
    case COMM_MGR_STM_CMD_SET_MODE:
        return MotorMgr_SetMode((MotorMgr_Mode)value);
    case COMM_MGR_STM_CMD_SET_SPEED_RPM:
        return MotorMgr_SetSpeedRpm(value);
    case COMM_MGR_STM_CMD_SET_POSITION_CDEG:
        return MotorMgr_SetPositionCdeg(value);
    case COMM_MGR_STM_CMD_START:
        return MotorMgr_Start();
    case COMM_MGR_STM_CMD_STOP:
        return MotorMgr_Stop();
    case COMM_MGR_STM_CMD_ACK_FAULT:
        return MotorMgr_AcknowledgeFault();
    case COMM_MGR_STM_CMD_ZERO_POSITION:
    default:
        /* Compatibility command is deliberately rejected: never rewrite the estimator zero. */
        return false;
    }
}

void CommMgr_STM_Task(void)
{
    static uint32_t last_telemetry;
    const uint32_t now = HAL_GetTick();

    CAN_STM_Task();
    USART_STM_Task();
    if ((uint32_t)(now - last_telemetry) >= COMM_MGR_STM_TELEMETRY_MS)
    {
        MotorMgr_State state;
        last_telemetry = now;
        MotorMgr_GetState(&state);
        CAN_STM_SendState(&state);
        USART_STM_SendState(&state);
    }
}
