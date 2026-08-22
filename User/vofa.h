#ifndef __VOFA_H
#define __VOFA_H

#include "main.h"

/* 初始化 */
void VOFA_Init(void);

/* 周期任务，在while(1)调用 */
void VOFA_Task(void);

#endif