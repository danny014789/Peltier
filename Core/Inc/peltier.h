/**
  ******************************************************************************
  * @file    peltier.h
  * @brief   DRV592 H-bridge driver. Two complementary PWMs on TIM1_CH1/CH2,
  *          plus /SHUTDOWN, HI-Z, and /FAULT0,1 GPIOs.
  *
  *          Output convention: signed duty in [-PELTIER_DUTY_MAX, +max]
  *          where positive = "heat" (current direction IN+), negative
  *          = "cool" (current direction IN-). Idle = both PWM outputs 0
  *          (coast).
  ******************************************************************************
  */
#ifndef PELTIER_H
#define PELTIER_H

#include <stdint.h>

#define PELTIER_DUTY_MAX  1000   /* user-facing duty unit: +/-1000 = +/-100% */

void    Peltier_Init(void);
void    Peltier_Enable(uint8_t enable);
void    Peltier_SetOutput(int16_t duty);
int16_t Peltier_GetDuty(void);
uint8_t Peltier_FaultActive(void);
uint8_t Peltier_IsEnabled(void);

#endif /* PELTIER_H */
