/**
  ******************************************************************************
  * @file    control.c
  * @brief   Closed-loop temperature controller. Sampling, PID, actuation,
  *          fault handling, and status LED heartbeat.
  ******************************************************************************
  */
#include "control.h"
#include "main.h"
#include "ntc.h"
#include "peltier.h"

extern TIM_HandleTypeDef htim3;

static PID_t          s_pid;
static ControlState_t s_state;

static void apply_setpoint_clamp(float *sp)
{
    if (*sp < CTRL_SETPOINT_MIN_C) *sp = CTRL_SETPOINT_MIN_C;
    if (*sp > CTRL_SETPOINT_MAX_C) *sp = CTRL_SETPOINT_MAX_C;
}

void Control_Init(void)
{
    NTC_Init();
    Peltier_Init();

    s_state.setpoint_c = CTRL_DEFAULT_SETPOINT;
    s_state.temp_c     = 0.0f;
    s_state.duty       = 0;
    s_state.enabled    = 0;
    s_state.fault      = 0;
    s_state.sensor_ok  = 0;
    s_state.tick_count = 0;

    apply_setpoint_clamp(&s_state.setpoint_c);

    PID_Init(&s_pid,
             CTRL_DEFAULT_KP, CTRL_DEFAULT_KI, CTRL_DEFAULT_KD,
             CTRL_DT_S,
             -(float)PELTIER_DUTY_MAX, (float)PELTIER_DUTY_MAX);
    PID_SetSetpoint(&s_pid, s_state.setpoint_c);

    /* Kick off the 50 Hz periodic interrupt. */
    HAL_TIM_Base_Start_IT(&htim3);
}

void Control_SetEnabled(uint8_t enable)
{
    if (enable) {
        if (s_state.fault) return;             /* must clear fault first */
        PID_Reset(&s_pid);
        Peltier_Enable(1);
        s_state.enabled = 1;
    } else {
        Peltier_SetOutput(0);
        Peltier_Enable(0);
        s_state.enabled = 0;
    }
}

void Control_SetSetpoint(float setpoint_c)
{
    apply_setpoint_clamp(&setpoint_c);
    s_state.setpoint_c = setpoint_c;
    PID_SetSetpoint(&s_pid, setpoint_c);
}

void Control_ClearFault(void)
{
    s_state.fault = 0;
}

const ControlState_t *Control_GetState(void) { return &s_state; }
PID_t                *Control_GetPID(void)   { return &s_pid;   }

/**
  * @brief 50 Hz tick — runs from TIM3 IRQ.
  *        Keeps work bounded: ADC poll + float math + PWM update.
  *        With ADC=12 MHz and 239.5+12.5 cycle sample, one conversion
  *        ~21 us; 8 averages ~170 us. Total tick well under 1 ms.
  */
void Control_Tick50Hz(void)
{
    s_state.tick_count++;

    /* Status LED heartbeat at 1 Hz (PC13 active-low) */
    if ((s_state.tick_count % CTRL_LOOP_HZ) == 0) {
        HAL_GPIO_TogglePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin);
    }

    float t = NTC_ReadTemperature();
    if (t <= CTRL_TEMP_MIN_C || t >= CTRL_TEMP_MAX_C || t == NTC_TEMP_INVALID) {
        s_state.sensor_ok = 0;
        s_state.fault = 1;
    } else {
        s_state.sensor_ok = 1;
        s_state.temp_c = t;
    }

    /* Hardware fault from DRV592 latches the system off. */
    if (Peltier_FaultActive()) {
        s_state.fault = 1;
    }

    if (s_state.fault) {
        if (s_state.enabled) {
            Peltier_SetOutput(0);
            Peltier_Enable(0);
            s_state.enabled = 0;
        }
        s_state.duty = 0;
        return;
    }

    if (!s_state.enabled || !s_state.sensor_ok) {
        Peltier_SetOutput(0);
        s_state.duty = 0;
        return;
    }

    float u = PID_Update(&s_pid, s_state.temp_c);
    int16_t duty = (int16_t)u;
    Peltier_SetOutput(duty);
    s_state.duty = duty;
}
