/**
  ******************************************************************************
  * @file    ntc.h
  * @brief   NTC thermistor reading via ADC1 + temperature conversion.
  *          Sensor: Semitec 103JT (R25 = 10 kohm, B25/85 = 3435 K).
  *          Topology: 3.3V --[10k]-- ADC node --[NTC]-- GND.
  ******************************************************************************
  */
#ifndef NTC_H
#define NTC_H

#include <stdint.h>

#define NTC_TEMP_INVALID  (-273.15f)

void     NTC_Init(void);
float    NTC_ReadTemperature(void);   /* deg C, blocking ~1 ms */
uint16_t NTC_GetLastRaw(void);

#endif /* NTC_H */
