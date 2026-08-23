#include "motor_mgr.h"

#include <limits.h>
#include <string.h>

#include "SguanFOC.h"
#include "UserData_Function.h"
#include "main.h"

#define MOTOR_MGR_RAD_PER_RPM          (0.10471975511965977f)
#define MOTOR_MGR_CDEG_PER_RAD         (5729.577951308232f)
#define MOTOR_MGR_RAD_PER_CDEG         (0.00017453292519943296)
#define MOTOR_MGR_STOP_SPEED_RAD_S     (10.0f * MOTOR_MGR_RAD_PER_RPM)
#define MOTOR_MGR_STOP_CONFIRM_MS      (100U)

typedef struct
{
    MotorMgr_Mode mode;
    MotorMgr_Mode pending_mode;
    MotorMgr_RunState state;
    bool initialized;
    bool output_enabled;
    bool start_requested;
    bool mode_change_pending;
    bool resume_after_mode_change;
    bool position_command_pending;
    bool below_stop_speed;
    uint32_t below_stop_since;
    float speed_target_rad_s;
    double position_target_rad;
} MotorMgr_Context;

static MotorMgr_Context s_motor;

static int16_t MotorMgr_ClampS16(int32_t value)
{
    if (value > INT16_MAX) return INT16_MAX;
    if (value < INT16_MIN) return INT16_MIN;
    return (int16_t)value;
}

static int32_t MotorMgr_NormalizeCdeg(int32_t value)
{
    value %= 36000;
    return (value < 0) ? value + 36000 : value;
}

static uint32_t MotorMgr_EnterCritical(void)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void MotorMgr_ExitCritical(uint32_t primask)
{
    if (primask == 0U) __enable_irq();
}

static double MotorMgr_ReadPositionRad(void)
{
    const uint32_t primask = MotorMgr_EnterCritical();
    const double position = Sguan.encoder.Real_Pos;
    MotorMgr_ExitCritical(primask);
    return position;
}

static void MotorMgr_WritePositionTarget(double position)
{
    const uint32_t primask = MotorMgr_EnterCritical();
    Sguan.foc.Target_Pos = position;
    MotorMgr_ExitCritical(primask);
}

static bool MotorMgr_IsSgFault(void)
{
    return Sguan.status >= MOTOR_STATUS_OVERVOLTAGE;
}

static bool MotorMgr_IsSgOperational(void)
{
    return (Sguan.status > MOTOR_STATUS_CALIBRATING) &&
           (Sguan.status < MOTOR_STATUS_OVERVOLTAGE);
}

static void MotorMgr_ResetControllers(void)
{
    Sguan.foc.Target_Id = 0.0f;
    Sguan.foc.Target_Iq = 0.0f;
    Sguan.foc.Ud_in = 0.0f;
    Sguan.foc.Uq_in = 0.0f;
    PID_Init(&Sguan.control.Current_D);
    PID_Init(&Sguan.control.Current_Q);
#if Open_PI_Control
    PID_Init(&Sguan.control.Velocity);
#else
    Ladrc_Init(&Sguan.control.Speed);
#endif
    PID_Init(&Sguan.control.Position);
}

static void MotorMgr_ApplyModeAndTargets(void)
{
    Sguan.mode = (s_motor.mode == MOTOR_MGR_MODE_POSITION) ?
        PosVelCur_THREE_MODE : VelCur_DOUBLE_MODE;
    Sguan.foc.Target_Id = 0.0f;
    Sguan.foc.Target_Pos = (s_motor.mode == MOTOR_MGR_MODE_POSITION) ?
        s_motor.position_target_rad : Sguan.encoder.Real_Pos;
    Sguan.foc.Target_Speed = (s_motor.mode == MOTOR_MGR_MODE_SPEED) ?
        s_motor.speed_target_rad_s : 0.0f;
}

static void MotorMgr_DisableOutput(void)
{
    const uint32_t primask = MotorMgr_EnterCritical();
    User_PWM_OutputDisable();
    s_motor.output_enabled = false;
    Sguan.foc.Target_Speed = 0.0f;
    Sguan.foc.Target_Pos = Sguan.encoder.Real_Pos;
    MotorMgr_ResetControllers();
    User_PwmDuty_Set(0U, 0U, 0U);
    MotorMgr_ExitCritical(primask);
}

static void MotorMgr_EnableOutput(void)
{
    const uint32_t primask = MotorMgr_EnterCritical();
    const uint16_t neutral = (uint16_t)(Sguan.motor.Duty / 2U);

    User_PWM_OutputDisable();
    MotorMgr_ResetControllers();
    Sguan.foc.Target_Pos = Sguan.encoder.Real_Pos;
    Sguan.foc.Target_Speed = 0.0f;
    User_PwmDuty_Set(neutral, neutral, neutral);
    User_PWM_OutputEnable();
    s_motor.output_enabled = true;
    MotorMgr_ApplyModeAndTargets();
    MotorMgr_ExitCritical(primask);
}

static void MotorMgr_FinishStop(void)
{
    bool restart = s_motor.start_requested;

    MotorMgr_DisableOutput();
    s_motor.below_stop_speed = false;
    s_motor.state = MOTOR_MGR_STOPPED;

    if (s_motor.mode_change_pending)
    {
        restart = restart || s_motor.resume_after_mode_change;
        s_motor.mode = s_motor.pending_mode;
        if ((s_motor.mode == MOTOR_MGR_MODE_POSITION) &&
            !s_motor.position_command_pending)
            s_motor.position_target_rad = MotorMgr_ReadPositionRad();
        s_motor.mode_change_pending = false;
        s_motor.resume_after_mode_change = false;
        s_motor.position_command_pending = false;
    }
    if (restart)
    {
        s_motor.start_requested = true;
        s_motor.state = MOTOR_MGR_STARTING;
    }
}

void MotorMgr_Init(void)
{
    memset(&s_motor, 0, sizeof(s_motor));
    s_motor.mode = MOTOR_MGR_MODE_SPEED;
    s_motor.pending_mode = MOTOR_MGR_MODE_SPEED;
    s_motor.state = MOTOR_MGR_STOPPED;
    s_motor.position_target_rad = MotorMgr_ReadPositionRad();
    Sguan.status = MOTOR_STATUS_STANDBY;
    User_PWM_OutputDisable();
}

bool MotorMgr_Start(void)
{
    if (MotorMgr_IsSgFault()) return false;
    s_motor.start_requested = true;
    if (s_motor.state == MOTOR_MGR_STOPPED) s_motor.state = MOTOR_MGR_STARTING;
    return true;
}

bool MotorMgr_Stop(void)
{
    s_motor.start_requested = false;
    s_motor.resume_after_mode_change = false;
    if ((s_motor.state == MOTOR_MGR_RUNNING) ||
        (s_motor.state == MOTOR_MGR_STARTING))
    {
        s_motor.below_stop_speed = false;
        s_motor.state = MOTOR_MGR_STOPPING;
    }
    return true;
}

bool MotorMgr_SetMode(MotorMgr_Mode mode)
{
    if ((mode != MOTOR_MGR_MODE_SPEED) &&
        (mode != MOTOR_MGR_MODE_POSITION)) return false;
    if ((mode == s_motor.mode) && !s_motor.mode_change_pending) return true;

    if ((s_motor.state == MOTOR_MGR_RUNNING) ||
        (s_motor.state == MOTOR_MGR_STARTING) ||
        (s_motor.state == MOTOR_MGR_STOPPING))
    {
        if (mode == MOTOR_MGR_MODE_POSITION)
        {
            s_motor.position_target_rad = MotorMgr_ReadPositionRad();
            s_motor.position_command_pending = false;
        }
        else
            s_motor.speed_target_rad_s = 0.0f;
        s_motor.pending_mode = mode;
        s_motor.mode_change_pending = true;
        s_motor.resume_after_mode_change = s_motor.output_enabled || s_motor.start_requested;
        s_motor.start_requested = false;
        s_motor.below_stop_speed = false;
        s_motor.state = MOTOR_MGR_STOPPING;
        return true;
    }

    s_motor.mode = mode;
    s_motor.pending_mode = mode;
    if (mode == MOTOR_MGR_MODE_POSITION)
        s_motor.position_target_rad = MotorMgr_ReadPositionRad();
    return true;
}

bool MotorMgr_SetSpeedRpm(int32_t speed_rpm)
{
    if ((speed_rpm > INT16_MAX) || (speed_rpm < INT16_MIN)) return false;
    s_motor.speed_target_rad_s = (float)speed_rpm * MOTOR_MGR_RAD_PER_RPM;
    if ((s_motor.mode == MOTOR_MGR_MODE_SPEED) &&
        (s_motor.state == MOTOR_MGR_RUNNING))
        Sguan.foc.Target_Speed = s_motor.speed_target_rad_s;
    return true;
}

bool MotorMgr_SetPositionCdeg(int32_t target_cdeg)
{
    const double current_position = MotorMgr_ReadPositionRad();
    const int32_t current_cdeg = (int32_t)(current_position * MOTOR_MGR_CDEG_PER_RAD);
    int32_t delta = MotorMgr_NormalizeCdeg(target_cdeg) -
                    MotorMgr_NormalizeCdeg(current_cdeg);
    if (delta > 18000) delta -= 36000;
    else if (delta < -18000) delta += 36000;

    s_motor.position_target_rad = current_position +
                                  (double)delta * MOTOR_MGR_RAD_PER_CDEG;
    if (s_motor.mode_change_pending &&
        (s_motor.pending_mode == MOTOR_MGR_MODE_POSITION))
        s_motor.position_command_pending = true;
    if ((s_motor.mode == MOTOR_MGR_MODE_POSITION) &&
        (s_motor.state == MOTOR_MGR_RUNNING))
        MotorMgr_WritePositionTarget(s_motor.position_target_rad);
    return true;
}

bool MotorMgr_AcknowledgeFault(void)
{
    if (!MotorMgr_IsSgFault()) return true;
    if (s_motor.output_enabled || (s_motor.state != MOTOR_MGR_FAULT)) return false;
    Sguan.status = MOTOR_STATUS_IDLE;
    s_motor.state = MOTOR_MGR_STOPPED;
    return true;
}

void MotorMgr_Task(void)
{
    if (MotorMgr_IsSgFault())
    {
        if ((s_motor.state != MOTOR_MGR_FAULT) || s_motor.output_enabled)
            MotorMgr_DisableOutput();
        s_motor.start_requested = false;
        s_motor.mode_change_pending = false;
        s_motor.resume_after_mode_change = false;
        s_motor.state = MOTOR_MGR_FAULT;
        return;
    }

    switch (s_motor.state)
    {
    case MOTOR_MGR_STARTING:
        if (!s_motor.start_requested)
        {
            MotorMgr_FinishStop();
        }
        else if (!s_motor.initialized)
        {
            if (Sguan.status == MOTOR_STATUS_STANDBY)
                Sguan.status = MOTOR_STATUS_UNINITIALIZED;
            else if (MotorMgr_IsSgOperational())
            {
                s_motor.initialized = true;
                MotorMgr_EnableOutput();
                s_motor.state = MOTOR_MGR_RUNNING;
            }
        }
        else if (MotorMgr_IsSgOperational())
        {
            MotorMgr_EnableOutput();
            s_motor.state = MOTOR_MGR_RUNNING;
        }
        break;

    case MOTOR_MGR_RUNNING:
        if (!s_motor.start_requested)
        {
            s_motor.below_stop_speed = false;
            s_motor.state = MOTOR_MGR_STOPPING;
        }
        break;

    case MOTOR_MGR_STOPPING:
        Sguan.foc.Target_Speed = 0.0f;
        if (s_motor.mode == MOTOR_MGR_MODE_POSITION)
            MotorMgr_WritePositionTarget(MotorMgr_ReadPositionRad());
        if (Value_fabsf(Sguan.encoder.Real_Speed) < MOTOR_MGR_STOP_SPEED_RAD_S)
        {
            const uint32_t now = HAL_GetTick();
            if (!s_motor.below_stop_speed)
            {
                s_motor.below_stop_speed = true;
                s_motor.below_stop_since = now;
            }
            else if ((uint32_t)(now - s_motor.below_stop_since) >=
                     MOTOR_MGR_STOP_CONFIRM_MS)
                MotorMgr_FinishStop();
        }
        else
            s_motor.below_stop_speed = false;
        break;

    case MOTOR_MGR_STOPPED:
    case MOTOR_MGR_FAULT:
    default:
        break;
    }
}

void MotorMgr_GetState(MotorMgr_State *state)
{
    int32_t current_cdeg;
    int32_t target_cdeg;
    if (state == NULL) return;
    memset(state, 0, sizeof(*state));
    state->mode = s_motor.mode;
    state->run_state = s_motor.state;
    state->running = s_motor.output_enabled;
    state->fault = MotorMgr_IsSgFault();
    if (state->fault)
    {
        const uint8_t offset = (uint8_t)(Sguan.status - MOTOR_STATUS_OVERVOLTAGE);
        state->faults = (offset < 16U) ? (uint16_t)(1U << offset) : 0x8000U;
    }
    state->speed_rpm = MotorMgr_ClampS16((int32_t)(Sguan.encoder.Real_Speed / MOTOR_MGR_RAD_PER_RPM));
    state->speed_ref_rpm = MotorMgr_ClampS16((int32_t)(
        ((s_motor.mode == MOTOR_MGR_MODE_POSITION) ?
         Sguan.control.Position.run.Output : s_motor.speed_target_rad_s) /
        MOTOR_MGR_RAD_PER_RPM));
    current_cdeg = (int32_t)(MotorMgr_ReadPositionRad() * MOTOR_MGR_CDEG_PER_RAD);
    target_cdeg = (int32_t)(s_motor.position_target_rad * MOTOR_MGR_CDEG_PER_RAD);
    state->position_cdeg = (uint16_t)MotorMgr_NormalizeCdeg(current_cdeg);
    state->position_ref_cdeg = (uint16_t)MotorMgr_NormalizeCdeg(target_cdeg);
    state->position_error_cdeg = MotorMgr_ClampS16(target_cdeg - current_cdeg);
    state->iq_ma = MotorMgr_ClampS16((int32_t)(Sguan.current.Real_Iq * 1000.0f));
    state->id_ma = MotorMgr_ClampS16((int32_t)(Sguan.current.Real_Id * 1000.0f));
    state->iq_ref_ma = MotorMgr_ClampS16((int32_t)(Sguan.control.Velocity.run.Output * 1000.0f));
    state->id_ref_ma = MotorMgr_ClampS16((int32_t)(Sguan.foc.Target_Id * 1000.0f));
    state->uq_mv = MotorMgr_ClampS16((int32_t)(Sguan.foc.Uq_in * 1000.0f));
    state->ud_mv = MotorMgr_ClampS16((int32_t)(Sguan.foc.Ud_in * 1000.0f));
}
