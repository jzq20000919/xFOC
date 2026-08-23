#ifndef MOTOR_MGR_H
#define MOTOR_MGR_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    MOTOR_MGR_MODE_SPEED = 0,
    MOTOR_MGR_MODE_POSITION = 1
} MotorMgr_Mode;

typedef enum
{
    MOTOR_MGR_STOPPED = 0,
    MOTOR_MGR_STARTING,
    MOTOR_MGR_RUNNING,
    MOTOR_MGR_STOPPING,
    MOTOR_MGR_FAULT
} MotorMgr_RunState;

typedef struct
{
    MotorMgr_Mode mode;
    MotorMgr_RunState run_state;
    bool running;
    bool fault;
    uint16_t faults;
    int16_t speed_rpm;
    int16_t speed_ref_rpm;
    uint16_t position_cdeg;
    uint16_t position_ref_cdeg;
    int16_t position_error_cdeg;
    int16_t iq_ma;
    int16_t id_ma;
    int16_t iq_ref_ma;
    int16_t id_ref_ma;
    int16_t uq_mv;
    int16_t ud_mv;
} MotorMgr_State;

void MotorMgr_Init(void);
void MotorMgr_Task(void);
bool MotorMgr_Start(void);
bool MotorMgr_Stop(void);
bool MotorMgr_SetMode(MotorMgr_Mode mode);
bool MotorMgr_SetSpeedRpm(int32_t speed_rpm);
bool MotorMgr_SetPositionCdeg(int32_t position_cdeg);
bool MotorMgr_AcknowledgeFault(void);
void MotorMgr_GetState(MotorMgr_State *state);

#endif
