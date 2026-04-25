/**
  ******************************************************************************
  * @file    peltier.c
  * @brief   DRV592 driver. Drive scheme: drive-then-coast (one PWM at a time).
  *            heat (+):  IN+ = duty,   IN- = 0
  *            cool (-):  IN+ = 0,      IN- = duty
  *            zero:      both 0   (coast through outputs to GND via internal
  *                                  body diodes; HI-Z pin separately controls
  *                                  output tri-state)
  ******************************************************************************
  */
#include "peltier.h"
#include "main.h"

extern TIM_HandleTypeDef htim1;

#define TIM1_PERIOD_TICKS  2400U                 /* ARR + 1 */

static int16_t s_duty;
static uint8_t s_enabled;

static inline uint32_t magnitude_to_ccr(uint16_t mag /* 0..PELTIER_DUTY_MAX */)
{
    if (mag > PELTIER_DUTY_MAX) mag = PELTIER_DUTY_MAX;
    return ((uint32_t)mag * TIM1_PERIOD_TICKS) / PELTIER_DUTY_MAX;
}

void Peltier_Init(void)
{
    s_duty = 0;
    s_enabled = 0;

    /* Start with PWMs at 0 then enable PWM generation. */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

    /* Default chip state: shutdown asserted, outputs high-Z. */
    HAL_GPIO_WritePin(DRV592_SHUTDOWN_GPIO_Port, DRV592_SHUTDOWN_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DRV592_HIZ_GPIO_Port,      DRV592_HIZ_Pin,      GPIO_PIN_RESET);
}

void Peltier_Enable(uint8_t enable)
{
    if (enable) {
        HAL_GPIO_WritePin(DRV592_SHUTDOWN_GPIO_Port, DRV592_SHUTDOWN_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(DRV592_HIZ_GPIO_Port,      DRV592_HIZ_Pin,      GPIO_PIN_SET);
        s_enabled = 1;
    } else {
        Peltier_SetOutput(0);
        HAL_GPIO_WritePin(DRV592_HIZ_GPIO_Port,      DRV592_HIZ_Pin,      GPIO_PIN_RESET);
        HAL_GPIO_WritePin(DRV592_SHUTDOWN_GPIO_Port, DRV592_SHUTDOWN_Pin, GPIO_PIN_RESET);
        s_enabled = 0;
    }
}

void Peltier_SetOutput(int16_t duty)
{
    if (duty >  PELTIER_DUTY_MAX) duty =  PELTIER_DUTY_MAX;
    if (duty < -PELTIER_DUTY_MAX) duty = -PELTIER_DUTY_MAX;
    s_duty = duty;

    if (duty > 0) {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, magnitude_to_ccr((uint16_t)duty));
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    } else if (duty < 0) {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, magnitude_to_ccr((uint16_t)(-duty)));
    } else {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    }
}

int16_t Peltier_GetDuty(void)        { return s_duty; }
uint8_t Peltier_IsEnabled(void)      { return s_enabled; }

uint8_t Peltier_FaultActive(void)
{
    /* Active-low open-drain with MCU pull-up. */
    if (HAL_GPIO_ReadPin(DRV592_FAULT0_GPIO_Port, DRV592_FAULT0_Pin) == GPIO_PIN_RESET) return 1;
    if (HAL_GPIO_ReadPin(DRV592_FAULT1_GPIO_Port, DRV592_FAULT1_Pin) == GPIO_PIN_RESET) return 1;
    return 0;
}
