/**
  ******************************************************************************
  * @file    pid.c
  * @brief   PID controller. Derivative-on-measurement (no setpoint kick).
  *          Anti-windup by conditional integration: integral only updates
  *          when the unsaturated output stays within limits OR when the
  *          error would push the output back toward the linear region.
  ******************************************************************************
  */
#include "pid.h"

void PID_Init(PID_t *p, float kp, float ki, float kd,
              float dt, float out_min, float out_max)
{
    p->kp = kp;
    p->ki = ki;
    p->kd = kd;
    p->dt = dt;
    p->out_min = out_min;
    p->out_max = out_max;
    p->setpoint = 0.0f;
    PID_Reset(p);
}

void PID_SetGains(PID_t *p, float kp, float ki, float kd)
{
    p->kp = kp;
    p->ki = ki;
    p->kd = kd;
}

void PID_SetSetpoint(PID_t *p, float setpoint)
{
    p->setpoint = setpoint;
}

void PID_Reset(PID_t *p)
{
    p->integral = 0.0f;
    p->prev_meas = 0.0f;
    p->primed = 0;
}

float PID_Update(PID_t *p, float meas)
{
    float err = p->setpoint - meas;

    float dmeas;
    if (p->primed) {
        dmeas = (meas - p->prev_meas) / p->dt;
    } else {
        dmeas = 0.0f;
        p->primed = 1;
    }
    p->prev_meas = meas;

    /* Tentative integral update */
    float new_integral = p->integral + err * p->dt;

    float u = p->kp * err
            + p->ki * new_integral
            - p->kd * dmeas;        /* d-on-measurement => negative sign */

    /* Anti-windup: clamp output and only commit integral if not winding-up */
    if (u > p->out_max) {
        u = p->out_max;
        if (err <= 0) p->integral = new_integral;   /* error pulling back */
    } else if (u < p->out_min) {
        u = p->out_min;
        if (err >= 0) p->integral = new_integral;
    } else {
        p->integral = new_integral;
    }
    return u;
}
