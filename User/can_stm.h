#ifndef CAN_STM_H
#define CAN_STM_H

#include <stdbool.h>
#include "motor_mgr.h"

bool CAN_STM_Init(void);
void CAN_STM_Task(void);
void CAN_STM_SendState(const MotorMgr_State *state);

#endif
