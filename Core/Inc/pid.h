/**
  ******************************************************************************
  * @file    pid.h
  * @brief   Discrete PID with derivative-on-measurement and conditional
  *          integration anti-windup. Output range and dt are caller-owned.
  ******************************************************************************
  */
#ifndef PID_H
#define PID_H

#include <stdint.h>

typedef struct {
    float   kp;
    float   ki;
    float   kd;
    float   setpoint;
    float   integral;
    float   prev_meas;
    float   out_min;
    float   out_max;
    float   dt;          /* seconds */
    uint8_t primed;      /* 0 until first sample seen */
} PID_t;

void  PID_Init(PID_t *p, float kp, float ki, float kd,
               float dt, float out_min, float out_max);
void  PID_SetGains(PID_t *p, float kp, float ki, float kd);
void  PID_SetSetpoint(PID_t *p, float setpoint);
void  PID_Reset(PID_t *p);
float PID_Update(PID_t *p, float measurement);

#endif /* PID_H */
