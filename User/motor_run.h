#ifndef __MOTOR_RUN_H
#define __MOTOR_RUN_H

#include <stdint.h>

typedef enum
{
    MOTOR_RUN_STOPPED = 0,
    MOTOR_RUN_STARTING,
    MOTOR_RUN_RUNNING,
    MOTOR_RUN_DECELERATING
} MotorRun_State;

void MotorRun_Init(void);
void MotorRun_RequestToggle(void);
void MotorRun_Task(void);

/* Used by the 20 kHz ADC callback to stop all closed-loop integration. */
uint8_t MotorRun_FocEnabled(void);

MotorRun_State MotorRun_GetState(void);
uint8_t MotorRun_IsInitialized(void);

#endif
