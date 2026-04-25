/**
  ******************************************************************************
  * @file    ntc.c
  * @brief   NTC thermistor (Semitec 103JT) ADC reader + Beta-equation
  *          conversion. ADC1 channel 0 on PA0, polled, 8-sample average.
  ******************************************************************************
  */
#include "ntc.h"
#include "main.h"
#include <math.h>

extern ADC_HandleTypeDef hadc1;

#define NTC_R25_OHM       10000.0f
#define NTC_BETA_K        3435.0f
#define NTC_T0_K          298.15f          /* 25 deg C */
#define NTC_RDIV_TOP_OHM  10000.0f         /* fixed resistor VDD -> ADC node */
#define ADC_FULL_SCALE    4095.0f
#define NTC_AVG_SAMPLES   8

static uint16_t s_last_raw;

void NTC_Init(void)
{
    HAL_ADCEx_Calibration_Start(&hadc1);
}

uint16_t NTC_GetLastRaw(void)
{
    return s_last_raw;
}

float NTC_ReadTemperature(void)
{
    uint32_t accum = 0;
    uint8_t  ok = 0;
    for (uint8_t i = 0; i < NTC_AVG_SAMPLES; ++i) {
        if (HAL_ADC_Start(&hadc1) != HAL_OK) continue;
        if (HAL_ADC_PollForConversion(&hadc1, 5) == HAL_OK) {
            accum += HAL_ADC_GetValue(&hadc1);
            ok++;
        }
    }
    HAL_ADC_Stop(&hadc1);

    if (ok == 0) {
        return NTC_TEMP_INVALID;
    }

    uint16_t raw = (uint16_t)(accum / ok);
    s_last_raw = raw;

    /* Open / short detection */
    if (raw < 8 || raw > (ADC_FULL_SCALE - 8.0f)) {
        return NTC_TEMP_INVALID;
    }

    /* V_node = VDD * raw / 4095 = VDD * R_ntc / (R_top + R_ntc)
     *   => R_ntc = R_top * raw / (4095 - raw)                       */
    float r_ntc = NTC_RDIV_TOP_OHM * (float)raw / (ADC_FULL_SCALE - (float)raw);

    /* Beta equation: 1/T = 1/T0 + (1/B) * ln(R/R25) */
    float inv_t = 1.0f / NTC_T0_K + (1.0f / NTC_BETA_K) * logf(r_ntc / NTC_R25_OHM);
    float t_kelvin = 1.0f / inv_t;
    return t_kelvin - 273.15f;
}
