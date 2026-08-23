#ifndef USART_STM_H
#define USART_STM_H

#include <stdbool.h>
#include "motor_mgr.h"

bool USART_STM_Init(void);
void USART_STM_Task(void);
void USART_STM_SendState(const MotorMgr_State *state);

#endif
