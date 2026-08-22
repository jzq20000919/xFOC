#ifndef __KEY_H
#define __KEY_H

#include "main.h"

/* Physical mapping: SW1=start/stop, SW2=speed up, SW3=speed down. */
#define MOTOR_RPM_MAX                 2600
#define MOTOR_RPM_MIN                 0
#define MOTOR_RPM_STEP                100
#define MOTOR_START_RPM               1000
#define MOTOR_ACCEL_RPM_PER_SECOND    1000.0f

extern volatile int32_t g_target_rpm;
extern volatile float   g_target_speed_rad;
/* User run request only; the actual run state is owned by motor_run.c. */
extern volatile uint8_t g_motor_run;

void Key_ControlInit(void);
void Key_Task(void);

#endif
