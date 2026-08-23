#ifndef COMM_MGR_STM_H
#define COMM_MGR_STM_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    COMM_MGR_STM_CMD_NOP = 0,
    COMM_MGR_STM_CMD_SET_MODE = 1,
    COMM_MGR_STM_CMD_SET_SPEED_RPM = 2,
    COMM_MGR_STM_CMD_SET_POSITION_CDEG = 3,
    COMM_MGR_STM_CMD_START = 4,
    COMM_MGR_STM_CMD_STOP = 5,
    COMM_MGR_STM_CMD_ACK_FAULT = 6,
    COMM_MGR_STM_CMD_ZERO_POSITION = 7,
    COMM_MGR_STM_CMD_PING = 8
} CommMgr_STM_Command;

void CommMgr_STM_Init(void);
void CommMgr_STM_Task(void);
bool CommMgr_STM_HandleCommand(uint8_t command, int32_t value);

#endif
