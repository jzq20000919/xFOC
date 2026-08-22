#ifndef __KEY_H
#define __KEY_H

#include "main.h"

#define MOTOR_RPM_MAX       2600
#define MOTOR_RPM_MIN       -2600
#define MOTOR_RPM_STEP      100

extern volatile int32_t g_target_rpm;
extern volatile float   g_target_speed_rad;
extern volatile uint8_t g_motor_run;

void Key_ControlInit(void);
void Key_Task(void);

#endif