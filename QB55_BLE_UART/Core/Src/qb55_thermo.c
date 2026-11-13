/**
  * Temperature control, see qb55_thermo.h.
  *
  * Structure: ADC + thermistor -> filtered temperature -> PID -> IN+ (qb55_in_out.c). A simulated
  * thermal plant can stand in for the ADC ("sim on") so the whole loop can be exercised with no
  * hardware connected; it feeds a raw ADC count through the SAME conversion and fault checks as
  * the real sensor, so the thermistor maths and fault handling are tested too.
  */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "main.h"
#include "app_common.h"
#include "stm32_seq.h"
#include "hw_if.h"
#include "qb55_in_out.h"
#include "qb55_thermo.h"

/* ---- ADC ---------------------------------------------------------------------------------- */

#define ADC_OVERSAMPLE     16U                       /* hardware averaging, see AdcInit() */
#define ADC_FULL_SCALE     (4095U * ADC_OVERSAMPLE)  /* the oversampled result is a 16-bit sum */
#define ADC_FAULT_MARGIN   100U                      /* this close to either rail = open/short */
#define SENSOR_FAIL_LIMIT  5U                        /* consecutive bad readings before a fault */

static ADC_HandleTypeDef hadc1;
static uint8_t adc_ready;

void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc)
{
  GPIO_InitTypeDef gpio = {0};

  if (hadc->Instance == ADC1)
  {
    __HAL_RCC_ADC_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_3;                   /* PC3 = ADC1_IN4 */
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &gpio);
  }
}

static void AdcInit(void)
{
  ADC_ChannelConfTypeDef ch = {0};

  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc1.Init.OversamplingMode = ENABLE;      /* 16 conversions summed per reading, no shift */
  hadc1.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_16;
  hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_NONE;
  hadc1.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc1.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    return;
  }
  (void)HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

  ch.Channel = ADC_CHANNEL_4;
  ch.Rank = ADC_REGULAR_RANK_1;
  ch.SamplingTime = ADC_SAMPLETIME_92CYCLES_5;   /* the divider is a ~5 k source: sample slowly */
  ch.SingleDiff = ADC_SINGLE_ENDED;
  ch.OffsetNumber = ADC_OFFSET_NONE;
  ch.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &ch) != HAL_OK)
  {
    return;
  }
  adc_ready = 1U;
}

static uint8_t AdcRead(uint16_t *raw)
{
  if (!adc_ready || HAL_ADC_Start(&hadc1) != HAL_OK)
  {
    return 0U;
  }
  if (HAL_ADC_PollForConversion(&hadc1, 20U) != HAL_OK)
  {
    (void)HAL_ADC_Stop(&hadc1);
    return 0U;
  }
  *raw = (uint16_t)HAL_ADC_GetValue(&hadc1);
  (void)HAL_ADC_Stop(&hadc1);
  return 1U;
}

/* ---- thermistor ----------------------------------------------------------------------------- */

#define T0_KELVIN  (QB55_NTC_T0_C + 273.15f)

/* The divider is fed from the same 3V3 as the ADC reference, so the count/full-scale ratio is
   independent of the actual supply voltage. */
static float RatioToResistance(float ratio)
{
#if QB55_NTC_ON_LOW_SIDE
  return QB55_NTC_FIXED_OHM * ratio / (1.0f - ratio);
#else
  return QB55_NTC_FIXED_OHM * (1.0f - ratio) / ratio;
#endif
}

static float ResistanceToRatio(float r_ntc)
{
#if QB55_NTC_ON_LOW_SIDE
  return r_ntc / (r_ntc + QB55_NTC_FIXED_OHM);
#else
  return QB55_NTC_FIXED_OHM / (r_ntc + QB55_NTC_FIXED_OHM);
#endif
}

/* B-parameter equation: 1/T = 1/T0 + ln(R/R0) / B */
static float ResistanceToCelsius(float r)
{
  return 1.0f / (1.0f / T0_KELVIN + logf(r / QB55_NTC_R0_OHM) / QB55_NTC_BETA_K) - 273.15f;
}

static float CelsiusToResistance(float t_c)
{
  return QB55_NTC_R0_OHM * expf(QB55_NTC_BETA_K * (1.0f / (t_c + 273.15f) - 1.0f / T0_KELVIN));
}

/* ---- state ----------------------------------------------------------------------------------- */

typedef enum
{
  ST_OFF = 0,
  ST_RUN,
  ST_FAULT_SENSOR,
  ST_FAULT_OVERTEMP,
  ST_FAULT_DRIVER
} State_t;

#define TEMP_FILTER_ALPHA  0.3f    /* exponential smoothing of the reading, per 100 ms tick */
#define LOG_EVERY_TICKS    (1000U / QB55_PID_PERIOD_MS)

#define SIM_AMBIENT_C      25.0f   /* the simulated plant: first order, gain in degC per volt */
#define SIM_GAIN_C_PER_V   30.0f
#define SIM_TAU_S          10.0f

static struct
{
  State_t  state;
  uint8_t  heat;            /* 1: drive raises temperature, 0: drive lowers it */
  float    setpoint;        /* NAN until the user sets one */
  float    kp, ki, kd;      /* V/degC, V/(degC*s), V*s/degC */
  float    out_max_v;
  float    integ;           /* integral term in volts */
  float    pv_prev;         /* previous process value, for the derivative */
  float    temp_f;          /* filtered temperature */
  uint8_t  temp_valid;
  float    out_v;
  uint32_t last_ms;
  uint8_t  bad_count;
  uint16_t raw;
  float    r_ntc;
  uint8_t  log_on;
  uint8_t  log_div;
  uint8_t  sim_on;
  uint8_t  sim_fault;       /* 0 = none, 1 = thermistor open, 2 = shorted */
  uint8_t  sim_drv_fault;   /* simulator: pretend FAULTZ is low */
  float    sim_t;
  uint32_t noise;
} S;

static uint8_t tick_id;
static uint8_t tick_running;

/* ---- DRV595 enable / fault pins ------------------------------------------------------------- */

static uint8_t drv_on;          /* the controller wants the DRV595 enabled */
static uint8_t fault_count;     /* consecutive control ticks with FAULTZ low */

void QB55_Drv_Init(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /* Disabled state first (Hi-Z high, SDZ low), then make the pins outputs: no glitch to "enabled". */
  HAL_GPIO_WritePin(QB55_DRV_HIZ_PORT, QB55_DRV_HIZ_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(QB55_DRV_SDZ_PORT, QB55_DRV_SDZ_PIN, GPIO_PIN_RESET);
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  gpio.Pin = QB55_DRV_HIZ_PIN;
  HAL_GPIO_Init(QB55_DRV_HIZ_PORT, &gpio);
  gpio.Pin = QB55_DRV_SDZ_PIN;
  HAL_GPIO_Init(QB55_DRV_SDZ_PORT, &gpio);

  /* FAULTZ is open drain: the board needs a pull-up to 3V3; ours is a backup so it never floats. */
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_PULLUP;
  gpio.Pin = QB55_DRV_FAULTZ_PIN;
  HAL_GPIO_Init(QB55_DRV_FAULTZ_PORT, &gpio);
}

static void DrvPins(uint8_t enable)
{
  if (enable)
  {
    HAL_GPIO_WritePin(QB55_DRV_SDZ_PORT, QB55_DRV_SDZ_PIN, GPIO_PIN_SET);    /* leave shutdown */
    HAL_Delay(QB55_DRV_ON_DELAY_MS);                                         /* turn-on time */
    HAL_GPIO_WritePin(QB55_DRV_HIZ_PORT, QB55_DRV_HIZ_PIN, GPIO_PIN_RESET);  /* release the outputs */
  }
  else
  {
    HAL_GPIO_WritePin(QB55_DRV_HIZ_PORT, QB55_DRV_HIZ_PIN, GPIO_PIN_SET);    /* fast disable first */
    HAL_GPIO_WritePin(QB55_DRV_SDZ_PORT, QB55_DRV_SDZ_PIN, GPIO_PIN_RESET);  /* then shutdown (also clears a latched fault) */
  }
}

/* The simulator never touches the real driver: it only tracks the state, the pins stay disabled. */
static void DrvSet(uint8_t on)
{
  drv_on = on;
  fault_count = 0U;
  DrvPins((uint8_t)(on && !S.sim_on));
}

static uint8_t FaultzLow(void)
{
  if (S.sim_on)
  {
    return S.sim_drv_fault;
  }
  return (HAL_GPIO_ReadPin(QB55_DRV_FAULTZ_PORT, QB55_DRV_FAULTZ_PIN) == GPIO_PIN_RESET) ? 1U : 0U;
}

/* ---- reading a temperature ------------------------------------------------------------------ */

static uint16_t SimRaw(void)
{
  float r_ntc;
  float ratio;
  int32_t counts;

  if (S.sim_fault == 1U)
  {
    r_ntc = 1.0e9f;
  }
  else if (S.sim_fault == 2U)
  {
    r_ntc = 0.0f;
  }
  else
  {
    r_ntc = CelsiusToResistance(S.sim_t);
  }
  ratio = ResistanceToRatio(r_ntc);
  S.noise = S.noise * 1664525U + 1013904223U;                    /* small deterministic noise */
  counts = (int32_t)(ratio * (float)ADC_FULL_SCALE) + (int32_t)((S.noise >> 24) % 41U) - 20;
  if (counts < 0)
  {
    counts = 0;
  }
  if (counts > (int32_t)ADC_FULL_SCALE)
  {
    counts = (int32_t)ADC_FULL_SCALE;
  }
  return (uint16_t)counts;
}

/* One conversion -> degC. Returns 0 if the reading is unusable (ADC failure, open or shorted
   thermistor, or a temperature outside the plausible range). */
static uint8_t ReadTemperature(float *t_c)
{
  uint16_t raw;
  float r;
  float t;

  if (S.sim_on)
  {
    raw = SimRaw();
  }
  else if (!AdcRead(&raw))
  {
    return 0U;
  }
  S.raw = raw;
  if (raw < ADC_FAULT_MARGIN || raw > (ADC_FULL_SCALE - ADC_FAULT_MARGIN))
  {
    return 0U;
  }
  r = RatioToResistance((float)raw / (float)ADC_FULL_SCALE);
  t = ResistanceToCelsius(r);
  if (!(t > -40.0f && t < 150.0f))
  {
    return 0U;
  }
  S.r_ntc = r;
  *t_c = t;
  return 1U;
}

/* Refresh the displayed temperature once, outside the control tick. */
static void MeasureNow(void)
{
  float t;

  if (ReadTemperature(&t))
  {
    S.temp_f = S.temp_valid ? S.temp_f + TEMP_FILTER_ALPHA * (t - S.temp_f) : t;
    S.temp_valid = 1U;
  }
  else
  {
    S.temp_valid = 0U;
  }
}

/* ---- control loop --------------------------------------------------------------------------- */

static void UpdateTick(void)
{
  const uint8_t need = (S.state == ST_RUN) || S.sim_on || S.log_on || drv_on;

  if (need && !tick_running)
  {
    HW_TS_Start(tick_id, (QB55_PID_PERIOD_MS * 1000U) / CFG_TS_TICK_VAL);
    tick_running = 1U;
  }
  else if (!need && tick_running)
  {
    HW_TS_Stop(tick_id);
    tick_running = 0U;
  }
}

static void OutputOff(State_t new_state)
{
  DrvSet(0U);                                   /* outputs high-impedance before anything else */
  S.state = new_state;
  S.out_v = 0.0f;
  S.integ = 0.0f;
  (void)QB55_In_SetMilliVolts(QB55_IN_PLUS, 0U);
  QB55_In_SetLocked(QB55_IN_PLUS, 0U);
  UpdateTick();
}

static void ApplyOutput(float volts)
{
  const float ceiling = (S.out_max_v < (float)QB55_PID_OUT_MAX_MV / 1000.0f) ?
                        S.out_max_v : (float)QB55_PID_OUT_MAX_MV / 1000.0f;

  if (volts > ceiling)
  {
    volts = ceiling;
  }
  if (volts < 0.0f)
  {
    volts = 0.0f;
  }
  S.out_v = volts;
  (void)QB55_In_SetMilliVolts(QB55_IN_PLUS, (uint16_t)(volts * 1000.0f + 0.5f));
}

static void ControlStep(void)
{
  const uint32_t now = HAL_GetTick();
  float dt = (float)(now - S.last_ms) / 1000.0f;
  float t;

  S.last_ms = now;
  if (dt < 0.01f || dt > 1.0f)
  {
    dt = (float)QB55_PID_PERIOD_MS / 1000.0f;
  }

  if (drv_on)             /* DRV595 fault line: low = short / over-current / over-temp / PVCC */
  {
    if (!FaultzLow())
    {
      fault_count = 0U;
    }
    else if (++fault_count >= QB55_DRV_FAULT_TICKS)
    {
      OutputOff(ST_FAULT_DRIVER);
      QB55_Uart_Print("FAULT: DRV595 FAULTZ low (short/over-current/over-temp/PVCC). Outputs disabled, PID stopped");
      return;
    }
  }

  if (S.sim_on)           /* advance the simulated plant with the output of the previous step */
  {
    const float target = SIM_AMBIENT_C + (S.heat ? 1.0f : -1.0f) * SIM_GAIN_C_PER_V * S.out_v;

    S.sim_t += (target - S.sim_t) * (dt / SIM_TAU_S);
  }

  if (!ReadTemperature(&t))
  {
    S.temp_valid = 0U;
    if (S.state == ST_RUN && ++S.bad_count >= SENSOR_FAIL_LIMIT)
    {
      OutputOff(ST_FAULT_SENSOR);
      QB55_Uart_Print("FAULT: no valid temperature (sensor open/short?). PID stopped, IN+ = 0 V");
    }
    return;
  }
  S.bad_count = 0U;
  if (S.temp_valid)
  {
    S.temp_f += TEMP_FILTER_ALPHA * (t - S.temp_f);
  }
  else
  {
    S.temp_f = t;
    S.temp_valid = 1U;
  }

  if (S.state != ST_RUN)
  {
    return;
  }

  if (S.temp_f > QB55_CUTOFF_C)
  {
    OutputOff(ST_FAULT_OVERTEMP);
    QB55_Uart_Print("FAULT: over-temperature cutoff. PID stopped, IN+ = 0 V");
    return;
  }

  {
    /* Work in "positive means drive harder" terms so heating and cooling share the code. */
    const float pv = S.heat ? S.temp_f : -S.temp_f;
    const float sp = S.heat ? S.setpoint : -S.setpoint;
    const float umax = (S.out_max_v < (float)QB55_PID_OUT_MAX_MV / 1000.0f) ?
                       S.out_max_v : (float)QB55_PID_OUT_MAX_MV / 1000.0f;
    const float e = sp - pv;
    const float p = S.kp * e;
    const float d = -S.kd * (pv - S.pv_prev) / dt;    /* derivative of the measurement, no kick */
    float i_next = S.integ + S.ki * e * dt;
    const float u_unsat = p + i_next + d;

    S.pv_prev = pv;
    /* Anti-windup: stop integrating while already saturated high and still wanting more. */
    if (!(u_unsat > umax && e > 0.0f))
    {
      if (i_next > umax)
      {
        i_next = umax;
      }
      if (i_next < 0.0f)
      {
        i_next = 0.0f;
      }
      S.integ = i_next;
    }
    ApplyOutput(p + S.integ + d);
  }
}

static void ThermoTask(void)
{
  ControlStep();
  if (S.log_on && ++S.log_div >= LOG_EVERY_TICKS)
  {
    char line[96];
    char t[16];

    S.log_div = 0U;
    if (S.temp_valid)
    {
      snprintf(t, sizeof(t), "%.2f", (double)S.temp_f);
    }
    else
    {
      snprintf(t, sizeof(t), "n/a");
    }
    snprintf(line, sizeof(line), "T=%s SP=%.2f OUT=%.3fV %s", t,
             isnan(S.setpoint) ? 0.0 : (double)S.setpoint, (double)S.out_v,
             (S.state == ST_RUN) ? "RUN" : (S.state == ST_OFF) ? "off" : "FAULT");
    QB55_Uart_Print(line);
  }
}

static void TickCallback(void)        /* timer ISR: just hand over to the sequencer */
{
  UTIL_SEQ_SetTask(1U << CFG_TASK_THERMO_ID, CFG_SCH_PRIO_0);
}

void QB55_Thermo_Init(void)
{
  memset(&S, 0, sizeof(S));
  drv_on = 0U;                       /* the pins were put in the disabled state by QB55_Drv_Init() */
  fault_count = 0U;
  S.heat = QB55_PID_DEFAULT_HEAT;
  S.setpoint = NAN;
  S.kp = 0.10f;
  S.ki = 0.005f;
  S.kd = 0.0f;
  S.out_max_v = (float)QB55_PID_OUT_MAX_MV / 1000.0f;
  S.sim_t = SIM_AMBIENT_C;
  S.noise = 12345U;

  AdcInit();
  UTIL_SEQ_RegTask(1U << CFG_TASK_THERMO_ID, UTIL_SEQ_RFU, ThermoTask);
  (void)HW_TS_Create(CFG_TIM_PROC_ID_ISR, &tick_id, hw_ts_Repeated, TickCallback);
}

/* ---- commands ------------------------------------------------------------------------------- */

/* Does `cmd` start with `word` as a whole word? On success *arg points at what follows. */
static int Starts(const char *cmd, const char *word, const char **arg)
{
  const size_t n = strlen(word);
  const char *a;

  if (strncmp(cmd, word, n) != 0 || (cmd[n] != '\0' && cmd[n] != ' '))
  {
    return 0;
  }
  a = cmd + n;
  while (*a == ' ')
  {
    a++;
  }
  *arg = a;
  return 1;
}

/* A number with an optional unit letter after it ("40", "40.5", "40c"). */
static int ParseNumber(const char *s, char unit, float *out)
{
  char *end;
  float v;

  if (*s == '\0')
  {
    return 0;
  }
  v = strtof(s, &end);
  if (end == s)
  {
    return 0;
  }
  if (unit != '\0' && *end == unit)
  {
    end++;
  }
  while (*end == ' ')
  {
    end++;
  }
  if (*end != '\0' || v != v || v > 1.0e6f || v < -1.0e6f)
  {
    return 0;
  }
  *out = v;
  return 1;
}

/* One line: what the controller wants and what the three DRV595 pins really read. */
static void DrvLine(char *out, size_t n)
{
  snprintf(out, n, "DRV595: %s  SDZ=%d Hi-Z=%d FAULTZ=%s%s\r\n",
           drv_on ? "enabled" : "disabled",
           HAL_GPIO_ReadPin(QB55_DRV_SDZ_PORT, QB55_DRV_SDZ_PIN) == GPIO_PIN_SET ? 1 : 0,
           HAL_GPIO_ReadPin(QB55_DRV_HIZ_PORT, QB55_DRV_HIZ_PIN) == GPIO_PIN_SET ? 1 : 0,
           HAL_GPIO_ReadPin(QB55_DRV_FAULTZ_PORT, QB55_DRV_FAULTZ_PIN) == GPIO_PIN_SET ? "ok" : "LOW",
           S.sim_on ? " (sim: pins held disabled)" : "");
}

static void ReplyStatus(char *reply, size_t n)
{
  char t[16];
  char sp[16];
  char drv[96];
  const char *st;

  MeasureNow();
  if (S.temp_valid)
  {
    snprintf(t, sizeof(t), "%.2fC", (double)S.temp_f);
  }
  else
  {
    snprintf(t, sizeof(t), "n/a");
  }
  if (isnan(S.setpoint))
  {
    snprintf(sp, sizeof(sp), "n/a");
  }
  else
  {
    snprintf(sp, sizeof(sp), "%.2fC", (double)S.setpoint);
  }
  switch (S.state)
  {
    case ST_RUN:             st = "RUN"; break;
    case ST_FAULT_SENSOR:    st = "FAULT-SENSOR"; break;
    case ST_FAULT_OVERTEMP:  st = "FAULT-OVERTEMP"; break;
    case ST_FAULT_DRIVER:    st = "FAULT-DRV595"; break;
    default:                 st = "off"; break;
  }
  DrvLine(drv, sizeof(drv));
  snprintf(reply, n,
           "T=%s SP=%s OUT=%.3fV PID=%s %s\r\n"
           "kp=%.3f ki=%.4f kd=%.3f max=%.3fV src=%s raw=%u R=%.0f\r\n%s",
           t, sp, (double)S.out_v, st, S.heat ? "heat" : "cool",
           (double)S.kp, (double)S.ki, (double)S.kd, (double)S.out_max_v,
           S.sim_on ? "sim" : (adc_ready ? "adc" : "adc-FAILED"),
           (unsigned)S.raw, (double)S.r_ntc, drv);
}

static void ReplyError(char *reply, size_t n, const char *msg)
{
  snprintf(reply, n, "error: %s\r\n", msg);
}

static void HandlePid(const char *arg, char *reply, size_t n)
{
  const char *a;
  float v;

  if (*arg == '\0')
  {
    ReplyStatus(reply, n);
  }
  else if (strcmp(arg, "on") == 0)
  {
    float t;

    if (isnan(S.setpoint))
    {
      ReplyError(reply, n, "set a temperature first, e.g.  set 40");
    }
    else if (!ReadTemperature(&t))
    {
      ReplyError(reply, n, "no valid temperature reading (sensor open/short? or use  sim on)");
    }
    else
    {
      (void)QB55_In_SetMilliVolts(QB55_IN_PLUS, 0U);    /* the DRV595 sees 0 V when it is enabled */
      DrvSet(1U);
      if (FaultzLow())
      {
        DrvSet(0U);
        ReplyError(reply, n, "DRV595 reports a fault (FAULTZ low); outputs left disabled. "
                             "Check PVCC, the load and the wiring");
      }
      else
      {
        S.temp_f = t;
        S.temp_valid = 1U;
        S.integ = 0.0f;
        S.pv_prev = S.heat ? t : -t;
        S.bad_count = 0U;
        S.last_ms = HAL_GetTick();
        S.state = ST_RUN;
        QB55_In_SetLocked(QB55_IN_PLUS, 1U);
        UpdateTick();
        snprintf(reply, n, "PID on: %s, T=%.1fC -> SP=%.1fC, output max %.3f V, DRV595 %s\r\n",
                 S.heat ? "heat" : "cool", (double)t, (double)S.setpoint, (double)S.out_max_v,
                 S.sim_on ? "pins held disabled (sim)" : "enabled");
      }
    }
  }
  else if (strcmp(arg, "off") == 0)
  {
    OutputOff(ST_OFF);
    snprintf(reply, n, "PID off, IN+ = 0 V, DRV595 disabled\r\n");
  }
  else if (Starts(arg, "mode", &a))
  {
    if (S.state == ST_RUN)
    {
      ReplyError(reply, n, "send  pid off  before changing the mode");
    }
    else if (strcmp(a, "heat") == 0 || strcmp(a, "cool") == 0)
    {
      S.heat = (a[0] == 'h') ? 1U : 0U;
      snprintf(reply, n, "mode = %s (%s)\r\n", a,
               S.heat ? "drive raises the temperature" : "drive lowers the temperature");
    }
    else
    {
      ReplyError(reply, n, "use  pid mode heat  or  pid mode cool");
    }
  }
  else if (Starts(arg, "kp", &a) || Starts(arg, "ki", &a) || Starts(arg, "kd", &a) ||
           Starts(arg, "max", &a))
  {
    float *target;
    float hi;
    const char *name;

    if (arg[0] == 'k' && arg[1] == 'p')      { target = &S.kp;  hi = 100.0f;  name = "kp"; }
    else if (arg[0] == 'k' && arg[1] == 'i') { target = &S.ki;  hi = 100.0f;  name = "ki"; }
    else if (arg[0] == 'k')                  { target = &S.kd;  hi = 1000.0f; name = "kd"; }
    else { target = &S.out_max_v; hi = (float)QB55_PID_OUT_MAX_MV / 1000.0f; name = "max"; }

    if (*a == '\0')
    {
      snprintf(reply, n, "%s = %.4f\r\n", name, (double)*target);
    }
    else if (!ParseNumber(a, (arg[0] == 'm') ? 'v' : '\0', &v) || v < 0.0f || v > hi)
    {
      snprintf(reply, n, "error: %s must be a number from 0 to %.3f\r\n", name, (double)hi);
    }
    else
    {
      *target = v;
      snprintf(reply, n, "%s = %.4f\r\n", name, (double)v);
    }
  }
  else
  {
    ReplyError(reply, n, "pid on | off | mode heat|cool | kp|ki|kd <x> | max <V>");
  }
}

/* Manual control of the DRV595 enable pins, for testing the board without the PID. */
static void HandleDrv(const char *arg, char *reply, size_t n)
{
  if (*arg == '\0')
  {
    DrvLine(reply, n);
  }
  else if (strcmp(arg, "on") == 0)
  {
    if (QB55_In_GetMilliVolts(QB55_IN_PLUS) > QB55_PID_OUT_MAX_MV)
    {
      ReplyError(reply, n, "IN+ is above 1.000 V; lower it first (IN+ 0V) so the DRV595 does not start hard");
      return;
    }
    DrvSet(1U);
    if (FaultzLow())
    {
      DrvSet(0U);
      ReplyError(reply, n, "DRV595 reports a fault (FAULTZ low); outputs left disabled");
    }
    else
    {
      snprintf(reply, n, "DRV595 enabled%s: the outputs are live\r\n",
               S.sim_on ? " (simulator: pins held disabled)" : "");
    }
  }
  else if (strcmp(arg, "off") == 0)
  {
    if (S.state == ST_RUN)
    {
      OutputOff(ST_OFF);                         /* the PID cannot run without the driver */
    }
    else
    {
      DrvSet(0U);
      UpdateTick();
    }
    snprintf(reply, n, "DRV595 disabled (outputs high-impedance)\r\n");
  }
  else
  {
    ReplyError(reply, n, "drv | drv on | drv off");
  }
}

int QB55_Thermo_Command(const char *cmd, char *reply, size_t reply_len)
{
  const char *arg;
  const char *a;
  float v;

  if (strcmp(cmd, "status") == 0)
  {
    ReplyStatus(reply, reply_len);
  }
  else if (strcmp(cmd, "help") == 0)
  {
    snprintf(reply, reply_len,
             "set <C> | temp | pid on|off|mode|kp|ki|kd|max | drv [on|off] | status | "
             "log on|off | led on|off | IN+/IN- <V>\r\n");
  }
  else if (Starts(cmd, "drv", &arg))
  {
    HandleDrv(arg, reply, reply_len);
  }
  else if (Starts(cmd, "set", &arg) || Starts(cmd, "temp", &arg))
  {
    if (*arg == '\0')
    {
      MeasureNow();
      if (S.temp_valid)
      {
        snprintf(reply, reply_len, "T = %.2f C\r\n", (double)S.temp_f);
      }
      else
      {
        ReplyError(reply, reply_len, "no valid temperature reading");
      }
    }
    else if (!ParseNumber(arg, 'c', &v) || v < QB55_SET_MIN_C || v > QB55_SET_MAX_C)
    {
      snprintf(reply, reply_len, "error: setpoint must be %.1f to %.1f C\r\n",
               (double)QB55_SET_MIN_C, (double)QB55_SET_MAX_C);
    }
    else
    {
      S.setpoint = v;
      snprintf(reply, reply_len, "SP = %.1f C\r\n", (double)v);
    }
  }
  else if (Starts(cmd, "pid", &arg))
  {
    HandlePid(arg, reply, reply_len);
  }
  else if (Starts(cmd, "log", &arg))
  {
    if (strcmp(arg, "on") == 0 || strcmp(arg, "off") == 0)
    {
      S.log_on = (arg[1] == 'n') ? 1U : 0U;
      S.log_div = 0U;
      UpdateTick();
      snprintf(reply, reply_len, "log %s\r\n", arg);
    }
    else
    {
      ReplyError(reply, reply_len, "log on | log off");
    }
  }
  else if (Starts(cmd, "sim", &arg))
  {
    if (strcmp(arg, "on") == 0)
    {
      if (S.state == ST_RUN || drv_on)
      {
        OutputOff(ST_OFF);                           /* disable the real DRV595 before switching */
      }
      S.sim_on = 1U;
      S.sim_fault = 0U;
      S.sim_drv_fault = 0U;
      S.sim_t = SIM_AMBIENT_C;
      S.temp_valid = 0U;
      UpdateTick();
      snprintf(reply, reply_len, "simulator on: plant starts at %.1f C ambient\r\n",
               (double)SIM_AMBIENT_C);
    }
    else if (strcmp(arg, "off") == 0)
    {
      S.sim_on = 0U;
      S.sim_fault = 0U;
      S.sim_drv_fault = 0U;
      S.temp_valid = 0U;
      if (S.state == ST_RUN)
      {
        OutputOff(ST_OFF);
      }
      else
      {
        DrvSet(0U);                                  /* a simulated "drv on" must not carry over */
      }
      UpdateTick();
      snprintf(reply, reply_len, "simulator off: reading the real thermistor\r\n");
    }
    else if (S.sim_on && (strcmp(arg, "open") == 0 || strcmp(arg, "short") == 0 ||
                          strcmp(arg, "ok") == 0))
    {
      S.sim_fault = (arg[0] == 'o' && arg[1] == 'p') ? 1U : (arg[0] == 's') ? 2U : 0U;
      snprintf(reply, reply_len, "simulated thermistor: %s\r\n", arg);
    }
    else if (S.sim_on && (strcmp(arg, "drvfault") == 0 || strcmp(arg, "drvok") == 0))
    {
      S.sim_drv_fault = (arg[3] == 'f') ? 1U : 0U;
      snprintf(reply, reply_len, "simulated FAULTZ: %s\r\n", S.sim_drv_fault ? "LOW (fault)" : "high (ok)");
    }
    else if (S.sim_on && Starts(arg, "temp", &a) && ParseNumber(a, 'c', &v) &&
             v > -39.0f && v < 149.0f)
    {
      S.sim_t = v;                                   /* test hook: jump the simulated plant */
      snprintf(reply, reply_len, "simulated plant forced to %.1f C\r\n", (double)v);
    }
    else
    {
      ReplyError(reply, reply_len, "sim on | sim off | (while on) sim open|short|ok|temp <C>|drvfault|drvok");
    }
  }
  else
  {
    return 0;
  }
  return 1;
}
