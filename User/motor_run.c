#include "motor_run.h"

#include "Key.h"
#include "SguanFOC.h"
#include "UserData_Function.h"

#define MOTOR_STOP_SPEED_RAD_S       (10.0f * 0.104719755f)
#define MOTOR_STOP_CONFIRM_TIME_MS   100U

volatile uint8_t g_motor_run = 0U;

static volatile MotorRun_State s_state = MOTOR_RUN_STOPPED;
static volatile uint8_t s_foc_enabled = 0U;
static uint8_t s_motor_initialized = 0U;
static uint8_t s_speed_below_threshold = 0U;
static uint32_t s_speed_below_since_ms = 0U;

static uint32_t MotorRun_EnterCritical(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    return primask;
}

static void MotorRun_ExitCritical(uint32_t primask)
{
    if (primask == 0U)
    {
        __enable_irq();
    }
}

static void MotorRun_ResetControlState(void)
{
    Sguan.foc.Target_Speed = 0.0f;
    Sguan.foc.Target_Pos = Sguan.encoder.Real_Pos;
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

    BPF_Init(&Sguan.bpf.CurrentD);
    BPF_Init(&Sguan.bpf.CurrentQ);
    BPF_Init(&Sguan.bpf.Encoder);

    Sguan.current.Real_Id = 0.0f;
    Sguan.current.Real_Iq = 0.0f;
    Sguan.current.Real_Ia = 0.0f;
    Sguan.current.Real_Ib = 0.0f;
    Sguan.current.Real_Ic = 0.0f;
    Sguan.current.Real_Ialpha = 0.0f;
    Sguan.current.Real_Ibeta = 0.0f;

    Sguan.encoder.Real_Speed = 0.0f;
    Sguan.encoder.Real_Espeed = 0.0f;

    Sguan.foc.Duty_u = 0U;
    Sguan.foc.Duty_v = 0U;
    Sguan.foc.Duty_w = 0U;
    Sguan.foc.Du = 0.0f;
    Sguan.foc.Dv = 0.0f;
    Sguan.foc.Dw = 0.0f;
}

static void MotorRun_SynchronizeEncoder(void)
{
    float raw_angle = User_Encoder_ReadRad();
    float mechanical_angle = Value_normalize(
        (raw_angle - Sguan.encoder.Pos_offset) *
        (float)Sguan.motor.Encoder_Dir);

    /*
     * TIM3 continues counting while MOE is off.  Seed every PLL history value
     * from the current encoder angle so the first enabled sample has no stale
     * phase error or stale speed output.
     */
    Sguan.encoder.pll.is_position_mode =
        (Sguan.mode == PosVelCur_THREE_MODE) ? 1U : 0U;
    Sguan.encoder.pll.go.i = 0.0f;
    Sguan.encoder.pll.go.Xo = 0.0f;
    Sguan.encoder.pll.go.Yo = mechanical_angle;
    Sguan.encoder.pll.go.Error = 0.0f;
    Sguan.encoder.pll.go.OutWe = 0.0f;
    Sguan.encoder.pll.go.OutRe = mechanical_angle;

    Sguan.encoder.Real_Rad = raw_angle;
    Sguan.encoder.Real_Pos = mechanical_angle;
    Sguan.encoder.Real_Speed = 0.0f;
    Sguan.encoder.Real_Erad = Value_normalize(
        mechanical_angle * (float)Sguan.motor.Poles);
    Sguan.encoder.Real_Espeed = 0.0f;
    fast_sin_cos(Sguan.encoder.Real_Erad,
                 &Sguan.foc.sine,
                 &Sguan.foc.cosine);

    /* Do not command an old position if this project is later put in mode 3. */
    Sguan.foc.Target_Pos = Sguan.encoder.Real_Pos;
}

static void MotorRun_EnableAfterInitialization(void)
{
    uint32_t primask = MotorRun_EnterCritical();
    uint16_t neutral_duty;

    s_foc_enabled = 0U;
    User_PWM_OutputDisable();
    MotorRun_ResetControlState();
    MotorRun_SynchronizeEncoder();

    /* Equal phase duties are a zero line-to-line voltage before the first ISR. */
    neutral_duty = (uint16_t)(Sguan.motor.Duty / 2U);
    Sguan.foc.Duty_u = neutral_duty;
    Sguan.foc.Duty_v = neutral_duty;
    Sguan.foc.Duty_w = neutral_duty;
    User_PwmDuty_Set(neutral_duty, neutral_duty, neutral_duty);

    s_foc_enabled = 1U;
    User_PWM_OutputEnable();
    MotorRun_ExitCritical(primask);
}

static void MotorRun_DisableOutput(void)
{
    uint32_t primask = MotorRun_EnterCritical();

    /* Gate the ISR before touching any state, then remove gate drive via MOE. */
    s_foc_enabled = 0U;
    User_PWM_OutputDisable();
    MotorRun_ResetControlState();
    User_PwmDuty_Set(0U, 0U, 0U);

    MotorRun_ExitCritical(primask);
    s_speed_below_threshold = 0U;
    s_state = MOTOR_RUN_STOPPED;
}

void MotorRun_Init(void)
{
    uint32_t primask = MotorRun_EnterCritical();

    g_motor_run = 0U;
    s_state = MOTOR_RUN_STOPPED;
    s_foc_enabled = 0U;
    s_motor_initialized = 0U;
    s_speed_below_threshold = 0U;
    s_speed_below_since_ms = 0U;

    Sguan.status = MOTOR_STATUS_STANDBY;
    User_PWM_OutputDisable();
    MotorRun_ExitCritical(primask);
}

void MotorRun_RequestToggle(void)
{
    g_motor_run = (uint8_t)!g_motor_run;
}

void MotorRun_Task(void)
{
    uint32_t now;
    float speed;

    if ((s_motor_initialized != 0U) &&
        ((s_state == MOTOR_RUN_RUNNING) ||
         (s_state == MOTOR_RUN_DECELERATING)) &&
        (Sguan.status >= MOTOR_STATUS_OVERVOLTAGE))
    {
        /* A library fault is a hard stop; normal KEY1 stops remain ramped. */
        g_motor_run = 0U;
        MotorRun_DisableOutput();
        return;
    }

    switch (s_state)
    {
    case MOTOR_RUN_STOPPED:
        if (g_motor_run != 0U)
        {
            s_state = MOTOR_RUN_STARTING;
        }
        break;

    case MOTOR_RUN_STARTING:
        if (g_motor_run == 0U)
        {
            MotorRun_DisableOutput();
            break;
        }

        if (s_motor_initialized == 0U)
        {
            if (Sguan.status == MOTOR_STATUS_STANDBY)
            {
                /* This is the only path that requests the blocking calibration. */
                Sguan.status = MOTOR_STATUS_UNINITIALIZED;
            }

            if ((Sguan.status > MOTOR_STATUS_CALIBRATING) &&
                (Sguan.status < MOTOR_STATUS_OVERVOLTAGE))
            {
                s_motor_initialized = 1U;
                MotorRun_EnableAfterInitialization();
                s_state = MOTOR_RUN_RUNNING;
            }
        }
        else
        {
            if ((Sguan.status > MOTOR_STATUS_CALIBRATING) &&
                (Sguan.status < MOTOR_STATUS_OVERVOLTAGE))
            {
                /* The calibrated offset is retained; only runtime state is reset. */
                MotorRun_EnableAfterInitialization();
                s_state = MOTOR_RUN_RUNNING;
            }
            else
            {
                /* Never re-enable MOE while the library reports a fault. */
                g_motor_run = 0U;
                MotorRun_DisableOutput();
            }
        }
        break;

    case MOTOR_RUN_RUNNING:
        if (g_motor_run == 0U)
        {
            Sguan.foc.Target_Speed = 0.0f;
            s_speed_below_threshold = 0U;
            s_state = MOTOR_RUN_DECELERATING;
        }
        break;

    case MOTOR_RUN_DECELERATING:
        if (g_motor_run != 0U)
        {
            s_speed_below_threshold = 0U;
            s_state = MOTOR_RUN_RUNNING;
            break;
        }

        Sguan.foc.Target_Speed = 0.0f;
        speed = Value_fabsf(Sguan.encoder.Real_Speed);
        now = HAL_GetTick();

        if (speed < MOTOR_STOP_SPEED_RAD_S)
        {
            if (s_speed_below_threshold == 0U)
            {
                s_speed_below_threshold = 1U;
                s_speed_below_since_ms = now;
            }
            else if ((uint32_t)(now - s_speed_below_since_ms) >=
                     MOTOR_STOP_CONFIRM_TIME_MS)
            {
                MotorRun_DisableOutput();
            }
        }
        else
        {
            s_speed_below_threshold = 0U;
        }
        break;

    default:
        g_motor_run = 0U;
        MotorRun_DisableOutput();
        break;
    }
}

uint8_t MotorRun_FocEnabled(void)
{
    return s_foc_enabled;
}

MotorRun_State MotorRun_GetState(void)
{
    return s_state;
}

uint8_t MotorRun_IsInitialized(void)
{
    return s_motor_initialized;
}
