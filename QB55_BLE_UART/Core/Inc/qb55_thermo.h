/**
  * Temperature control: reads a 10 k NTC thermistor, runs a PID loop, and drives IN+ (which feeds
  * the DRV595 / Peltier). Everything is set over Bluetooth with text commands, see
  * QB55_Thermo_Command() in qb55_thermo.c.
  *
  *   NTC divider midpoint -> PC3 (ADC1_IN4, Arduino A4, CN7 pin 36)
  *   PID output           -> IN+ (PA0, via the PWM + RC filter in qb55_in_out.c)
  *
  * ASSUMPTIONS TO CONFIRM against the real hardware - each one is a single constant below.
  */
#ifndef QB55_THERMO_H
#define QB55_THERMO_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- thermistor and divider -------------------------------------------------------------- */
#define QB55_NTC_R0_OHM       10000.0f   /* resistance at QB55_NTC_T0_C */
#define QB55_NTC_T0_C         25.0f
/* ASSUMED: ~3435 K is typical for the Semitec 103JT family. Take the exact B value (and the
   temperature range it is quoted over) from your 103JT-050 datasheet. */
#define QB55_NTC_BETA_K       3435.0f
#define QB55_NTC_FIXED_OHM    10000.0f   /* the series resistor of the divider */
/* ASSUMED: which side of the divider the thermistor is on.
     1:  3V3 --[fixed]--+--[NTC]-- GND      (reading falls as it gets hotter)
     0:  3V3 --[NTC]--+--[fixed]-- GND      (reading rises as it gets hotter)          */
#define QB55_NTC_ON_LOW_SIDE  1

/* ---- limits (protection) ----------------------------------------------------------------- */
#define QB55_SET_MIN_C        0.0f       /* lowest / highest temperature that can be requested */
#define QB55_SET_MAX_C        30.0f
#define QB55_CUTOFF_C         85.0f      /* measured above this: output 0 V and latch a fault */

/* The PID output sent to IN+ never exceeds this, whatever is commanded. With the DRV595's ~26 dB
   gain (x20) this is the main guard for the Peltier. */
#define QB55_PID_OUT_MAX_MV   1000U

#define QB55_PID_PERIOD_MS    100U       /* control rate: 10 Hz */

/* Direction IN+ drives the Peltier at power-up (it is not saved, so this is what every reset
   gives): 1 = positive drive raises the temperature, 0 = it lowers it. The user's Peltier is used to
   cool below room temperature. "pid mode heat|cool" changes it at run time (while the PID is off). */
#define QB55_PID_DEFAULT_HEAT 0

/* ---- DRV595 control pins (TI SLOS808A) ----------------------------------------------------- */
/* SDZ   (DRV595 pin 2):  LOW = outputs high-impedance (shutdown), HIGH = enabled.
   Hi-Z  (DRV595 pin 12): HIGH = outputs high-impedance (fast disable), LOW = enabled.
   FAULTZ(DRV595 pin 3):  open drain, LOW = fault (short, over-current, over-temperature, PVCC out
                          of range). Needs a pull-up to 3V3 (the MCU pin also has its own).
   The firmware keeps the outputs disabled (Hi-Z high, SDZ low) from boot until "pid on" / "drv on". */
#define QB55_DRV_SDZ_PORT      GPIOA
#define QB55_DRV_SDZ_PIN       GPIO_PIN_3
#define QB55_DRV_HIZ_PORT      GPIOA
#define QB55_DRV_HIZ_PIN       GPIO_PIN_2
#define QB55_DRV_FAULTZ_PORT   GPIOC
#define QB55_DRV_FAULTZ_PIN    GPIO_PIN_6
#define QB55_DRV_ON_DELAY_MS   12U       /* wait after SDZ goes high; the datasheet turn-on time is 10 ms */
#define QB55_DRV_FAULT_TICKS   2U        /* consecutive control ticks with FAULTZ low before we act */

/* Early (from main): put SDZ / Hi-Z in the disabled state and set up FAULTZ as an input. */
void QB55_Drv_Init(void);

/* Called once the BLE stack is up. Control stays OFF and IN+ at 0 V until "pid on". */
void QB55_Thermo_Init(void);

/* Handle a trimmed, lower-cased command line. Returns 1 if it was one of ours (a reply, possibly
   several lines, is then in `reply`), 0 if not. */
int QB55_Thermo_Command(const char *cmd, char *reply, size_t reply_len);

/* Implemented by p2p_server_app.c: queue one line for the connected client (dropped if none). */
void QB55_Uart_Print(const char *line);

#ifdef __cplusplus
}
#endif

#endif /* QB55_THERMO_H */
