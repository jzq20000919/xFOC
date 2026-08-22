#include "Key.h"
#include "motor_run.h"

#define RPM_TO_RADS              0.104719755f
#define KEY_SCAN_PERIOD_MS       10U
#define KEY_STABLE_SAMPLE_COUNT  3U

typedef struct
{
    GPIO_PinState candidate;
    GPIO_PinState stable;
    uint8_t count;
} KeyDebounceState;

volatile int32_t g_target_rpm = MOTOR_START_RPM;
volatile float g_target_speed_rad =
    (float)MOTOR_START_RPM * RPM_TO_RADS;
static void Key_SetTargetRPM(int32_t rpm)
{
    if (rpm > MOTOR_RPM_MAX)
    {
        rpm = MOTOR_RPM_MAX;
    }
    if (rpm < MOTOR_RPM_MIN)
    {
        rpm = MOTOR_RPM_MIN;
    }

    g_target_rpm = rpm;
    g_target_speed_rad = (float)rpm * RPM_TO_RADS;
}

/* Return 1 only when a debounced low-to-high transition is accepted. */
static uint8_t Key_PressedEvent(KeyDebounceState *state,
                                GPIO_PinState sample)
{
    if (sample != state->candidate)
    {
        state->candidate = sample;
        state->count = 1U;
        return 0U;
    }

    if (state->count < KEY_STABLE_SAMPLE_COUNT)
    {
        state->count++;
    }

    if ((state->count >= KEY_STABLE_SAMPLE_COUNT) &&
        (state->stable != state->candidate))
    {
        state->stable = state->candidate;
        return (state->stable == GPIO_PIN_SET) ? 1U : 0U;
    }

    return 0U;
}

void Key_ControlInit(void)
{
    Key_SetTargetRPM(MOTOR_START_RPM);
}

void Key_Task(void)
{
    static uint32_t last_tick = 0U;
    static KeyDebounceState key1 =
        {GPIO_PIN_RESET, GPIO_PIN_RESET, 0U};
    static KeyDebounceState key2 =
        {GPIO_PIN_RESET, GPIO_PIN_RESET, 0U};
    static KeyDebounceState key3 =
        {GPIO_PIN_RESET, GPIO_PIN_RESET, 0U};
    uint32_t now = HAL_GetTick();

    if ((uint32_t)(now - last_tick) < KEY_SCAN_PERIOD_MS)
    {
        return;
    }
    last_tick = now;

    if (Key_PressedEvent(&key1,
                         HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin)))
    {
        MotorRun_RequestToggle();
    }

    if (Key_PressedEvent(&key2,
                         HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin)))
    {
        Key_SetTargetRPM(g_target_rpm + MOTOR_RPM_STEP);
    }

    if (Key_PressedEvent(&key3,
                         HAL_GPIO_ReadPin(KEY3_GPIO_Port, KEY3_Pin)))
    {
        Key_SetTargetRPM(g_target_rpm - MOTOR_RPM_STEP);
    }
}
