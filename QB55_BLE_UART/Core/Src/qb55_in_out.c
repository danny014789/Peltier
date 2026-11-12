/**
  * IN+ / IN- analog outputs: see qb55_in_out.h. The hardware is TIM2 PWM today; only the
  * "backend" section below would change for a real DAC.
  */
#include <stdio.h>
#include <string.h>

#include "main.h"
#include "app_common.h"
#include "stm32_lpm.h"
#include "qb55_in_out.h"

/* ---- backend: PWM on TIM2 (QB55_PWM_BITS bits) ---------------------------------------------- */

/* Duty counts run 0..PWM_STEPS; PWM_STEPS itself (> ARR) keeps the pin high for the whole period,
   which is how full scale (VDD) is reached. The PWM frequency is TIM2CLK / PWM_STEPS: 31.25 kHz at
   10 bits and 32 MHz (7.8 kHz at 12 bits). */
#define PWM_STEPS  (1U << QB55_PWM_BITS)

static uint32_t MvToCcr(uint16_t mv)
{
  return ((uint32_t)mv * PWM_STEPS + QB55_VDD_MV / 2U) / QB55_VDD_MV;   /* nearest step */
}

static uint16_t CcrToMv(uint32_t ccr)
{
  return (uint16_t)((ccr * QB55_VDD_MV + PWM_STEPS / 2U) / PWM_STEPS);
}

void QB55_In_Init(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_TIM2_CLK_ENABLE();

  /* Drive both pins low as plain outputs while the timer is set up, so they never float. */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_RESET);
  gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &gpio);

  /* PWM mode 1 on CH1 and CH2, preload on so a new duty takes effect cleanly at the next
     period; both start at duty 0, i.e. 0 V. */
  TIM2->PSC = 0U;
  TIM2->ARR = PWM_STEPS - 1U;
  TIM2->CCR1 = 0U;
  TIM2->CCR2 = 0U;
  TIM2->CCMR1 = TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1PE |
                TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2PE;
  TIM2->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;
  TIM2->EGR = TIM_EGR_UG;
  TIM2->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;

  /* The timer is already running at 0 % duty (pins low): hand the pins over to it. */
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(GPIOA, &gpio);
}

void QB55_In_HoldPowerState(void)
{
  UTIL_LPM_SetStopMode(1U << CFG_LPM_QB55_IN, UTIL_LPM_DISABLE);
  UTIL_LPM_SetOffMode(1U << CFG_LPM_QB55_IN, UTIL_LPM_DISABLE);
}

uint8_t QB55_In_SetMilliVolts(QB55_In_t ch, uint16_t mv)
{
  if (mv > QB55_VDD_MV)
  {
    return 1U;
  }
  if (ch == QB55_IN_PLUS)
  {
    TIM2->CCR1 = MvToCcr(mv);
  }
  else
  {
    TIM2->CCR2 = MvToCcr(mv);
  }
  return 0U;
}

static uint8_t g_locked[2];

void QB55_In_SetLocked(QB55_In_t ch, uint8_t locked)
{
  g_locked[ch == QB55_IN_PLUS ? 0 : 1] = locked;
}

uint16_t QB55_In_GetMilliVolts(QB55_In_t ch)
{
  return CcrToMv((ch == QB55_IN_PLUS) ? TIM2->CCR1 : TIM2->CCR2);
}

/* ---- command handling (independent of the backend) ------------------------------------------ */

typedef enum
{
  PARSE_OK = 0,
  PARSE_BAD = 1,     /* not a number / unknown unit / junk after it */
  PARSE_RANGE = 2    /* a number, but above full scale */
} Parse_t;

/* "<digits>[.<digits>][ ]*[v|mv]" -> millivolts, rounded to the nearest. No unit means volts.
   Integer arithmetic only: no floating-point code is pulled into the firmware. */
static Parse_t ParseMilliVolts(const char *s, uint32_t *mv_out)
{
  uint32_t whole = 0U;
  uint32_t frac = 0U;         /* first six fractional digits: millionths of the unit */
  uint8_t  whole_digits = 0U;
  uint8_t  frac_digits = 0U;
  uint8_t  used;
  uint8_t  in_millivolts = 0U;
  uint32_t mv;

  while (*s == ' ')
  {
    s++;
  }
  while (*s >= '0' && *s <= '9')
  {
    if (whole < 1000000U)     /* saturate; the range check below rejects it */
    {
      whole = whole * 10U + (uint32_t)(*s - '0');
    }
    whole_digits++;
    s++;
  }
  if (*s == '.')
  {
    s++;
    while (*s >= '0' && *s <= '9')
    {
      if (frac_digits < 6U)
      {
        frac = frac * 10U + (uint32_t)(*s - '0');
      }
      frac_digits++;
      s++;
    }
  }
  if (whole_digits == 0U && frac_digits == 0U)
  {
    return PARSE_BAD;
  }
  for (used = (frac_digits > 6U) ? 6U : frac_digits; used < 6U; used++)
  {
    frac *= 10U;
  }

  while (*s == ' ')
  {
    s++;
  }
  if (s[0] == 'm' && s[1] == 'v')
  {
    in_millivolts = 1U;
    s += 2;
  }
  else if (s[0] == 'v')
  {
    s += 1;
  }
  while (*s == ' ')
  {
    s++;
  }
  if (*s != '\0')
  {
    return PARSE_BAD;
  }

  if (in_millivolts)
  {
    if (whole > 4000U)
    {
      return PARSE_RANGE;
    }
    mv = whole + (frac + 500000U) / 1000000U;
  }
  else
  {
    if (whole > 4U)
    {
      return PARSE_RANGE;
    }
    mv = whole * 1000U + (frac + 500U) / 1000U;
  }
  if (mv > QB55_VDD_MV)
  {
    return PARSE_RANGE;
  }
  *mv_out = mv;
  return PARSE_OK;
}

static void ReportLevel(QB55_In_t ch, char *reply, size_t reply_len)
{
  const uint16_t mv = QB55_In_GetMilliVolts(ch);

  snprintf(reply, reply_len, "%s = %u.%03u V\r\n", (ch == QB55_IN_PLUS) ? "IN+" : "IN-",
           (unsigned)(mv / 1000U), (unsigned)(mv % 1000U));
}

int QB55_In_Command(const char *cmd, char *reply, size_t reply_len)
{
  QB55_In_t ch;
  const char *arg;
  uint32_t mv = 0U;

  if (strncmp(cmd, "in+", 3) == 0)
  {
    ch = QB55_IN_PLUS;
  }
  else if (strncmp(cmd, "in-", 3) == 0)
  {
    ch = QB55_IN_MINUS;
  }
  else
  {
    return 0;
  }

  arg = cmd + 3;
  while (*arg == ' ')
  {
    arg++;
  }

  if (*arg == '\0')                       /* "IN+" on its own: report the present level */
  {
    ReportLevel(ch, reply, reply_len);
    return 1;
  }

  if (g_locked[ch == QB55_IN_PLUS ? 0 : 1])
  {
    snprintf(reply, reply_len, "error: %s is under PID control; send  pid off  first\r\n",
             (ch == QB55_IN_PLUS) ? "IN+" : "IN-");
    return 1;
  }

  switch (ParseMilliVolts(arg, &mv))
  {
    case PARSE_OK:
      (void)QB55_In_SetMilliVolts(ch, (uint16_t)mv);
      ReportLevel(ch, reply, reply_len);  /* the level really produced, after rounding */
      break;
    case PARSE_RANGE:
      snprintf(reply, reply_len, "error: %s max is %u.%03u V\r\n",
               (ch == QB55_IN_PLUS) ? "IN+" : "IN-",
               (unsigned)(QB55_VDD_MV / 1000U), (unsigned)(QB55_VDD_MV % 1000U));
      break;
    default:
      snprintf(reply, reply_len, "error: use %s <volts>, e.g. %s 2V (0 to %u.%u V)\r\n",
               (ch == QB55_IN_PLUS) ? "IN+" : "IN-", (ch == QB55_IN_PLUS) ? "IN+" : "IN-",
               (unsigned)(QB55_VDD_MV / 1000U), (unsigned)((QB55_VDD_MV % 1000U) / 100U));
      break;
  }
  return 1;
}
