#ifndef __VOFA_H
#define __VOFA_H

#include "main.h"

extern volatile uint32_t g_vofa_tx_complete_count;
extern volatile uint32_t g_vofa_tx_error_count;
extern volatile uint32_t g_vofa_last_uart_error;

/* 初始化 */
void VOFA_Init(void);

/* 周期任务，在while(1)调用 */
void VOFA_Task(void);

#endif
