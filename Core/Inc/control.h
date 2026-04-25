/**
  ******************************************************************************
  * @file    control.h
  * @brief   Top-level temperature control loop. Runs at 50 Hz from TIM3 IRQ:
  *          NTC sample -> PID -> Peltier output, with fault & range checks.
  ******************************************************************************
  */
#ifndef CONTROL_H
#define CONTROL_H

#include <stdint.h>
#include "pid.h"

/* Setpoint clamp range (deg C) */
#define CTRL_SETPOINT_MIN_C   0.0f
#define CTRL_SETPOINT_MAX_C   25.0f

/* Sensor sanity range (deg C). Outside -> fault. */
#define CTRL_TEMP_MIN_C      -40.0f
#define CTRL_TEMP_MAX_C       80.0f

#define CTRL_LOOP_HZ          50
#define CTRL_DT_S             (1.0f / (float)CTRL_LOOP_HZ)

#define CTRL_DEFAULT_KP       20.0f
#define CTRL_DEFAULT_KI        0.5f
#define CTRL_DEFAULT_KD        0.0f
#define CTRL_DEFAULT_SETPOINT 25.0f

typedef struct {
    float    setpoint_c;
    float    temp_c;
    int16_t  duty;             /* last applied duty, +/-PELTIER_DUTY_MAX */
    uint8_t  enabled;          /* 1 = control loop driving Peltier */
    uint8_t  fault;            /* 1 = latched fault, output disabled */
    uint8_t  sensor_ok;        /* 1 = last NTC reading was valid */
    uint32_t tick_count;       /* counts of 50 Hz loop iterations */
} ControlState_t;

void                  Control_Init(void);
void                  Control_Tick50Hz(void);     /* call from TIM3 IRQ */
const ControlState_t *Control_GetState(void);
PID_t                *Control_GetPID(void);

void  Control_SetEnabled(uint8_t enable);
void  Control_SetSetpoint(float setpoint_c);
void  Control_ClearFault(void);

#endif /* CONTROL_H */
