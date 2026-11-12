/**
  * IN+ / IN- analog outputs, set over Bluetooth with commands such as "IN+ 2V".
  *
  * The STM32WB55 has NO DAC, so each output is a PWM signal from TIM2 (QB55_PWM_BITS bits). Put a simple RC
  * low-pass filter on each pin (e.g. 10 k + 1 uF) to turn it into a steady voltage; the pin
  * itself swings between 0 V and VDD. Everything that talks to the hardware is behind
  * QB55_In_SetMilliVolts / QB55_In_GetMilliVolts, so a real DAC chip can replace the PWM later
  * without touching the command handling.
  *
  *   IN+  -> PA0 (TIM2_CH1, CN7 pin 34)
  *   IN-  -> PA1 (TIM2_CH2, CN7 pin 32)
  */
#ifndef QB55_IN_OUT_H
#define QB55_IN_OUT_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
  QB55_IN_PLUS = 0,
  QB55_IN_MINUS = 1
} QB55_In_t;

/* PWM resolution in bits. More bits = finer steps but a slower carrier (TIM2CLK / 2^bits), so a
   given RC filter leaves proportionally more ripple. With a single 100 k + 100 nF stage the ripple
   (about 2.6 mV at 10 bits) is already about one step (3.2 mV), so 10 bits wastes nothing; 12 bits
   (0.8 mV steps, 7.8 kHz) only pays off with a stronger, e.g. two-stage, filter. */
#define QB55_PWM_BITS  10U

/* Full-scale output = the supply the PWM swings between. The Nucleo runs from 3.3 V; the real
   supply is a little off, so absolute accuracy is about +-2 % unless this is set to the
   measured VDD. */
#define QB55_VDD_MV  3300U

/* Configure the pins and timer. Both outputs start at 0 V. Call as early as possible. */
void QB55_In_Init(void);

/* Keep the MCU out of Stop/Standby, which would stop the timer and freeze the pins at a random
   level. Call after UTIL_LPM_Init(). */
void QB55_In_HoldPowerState(void);

/* Returns 0 on success, 1 if `mv` is above QB55_VDD_MV (output left unchanged). */
uint8_t QB55_In_SetMilliVolts(QB55_In_t ch, uint16_t mv);

/* The voltage actually being produced, which differs from the request by up to half a step. */
uint16_t QB55_In_GetMilliVolts(QB55_In_t ch);

/* While locked, the "IN+ <volts>" text command is refused (a controller owns the output). The C
   function QB55_In_SetMilliVolts is unaffected, so the controller itself can still drive it. */
void QB55_In_SetLocked(QB55_In_t ch, uint8_t locked);

/* Handle a command line that has already been trimmed and lower-cased, such as "in+ 2v".
   Returns 1 if it was an IN+/IN- command (a reply for the client is then in `reply`), else 0. */
int QB55_In_Command(const char *cmd, char *reply, size_t reply_len);

#ifdef __cplusplus
}
#endif

#endif /* QB55_IN_OUT_H */
