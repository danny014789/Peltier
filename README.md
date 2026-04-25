# Peltier PID Controller — STM32F103C8 + DRV592

Closed-loop temperature control of a Peltier (TEC) module driven by a TI
DRV592 H-bridge, sensed by a Semitec 103JT NTC thermistor, configured and
commanded over USB CDC.

## Hardware

| Block | Part | Notes |
|---|---|---|
| MCU | STM32F103C8Tx, LQFP48 | HSE 8 MHz, LSE 32.768 kHz, SYSCLK 48 MHz |
| Driver | TI DRV592 | Differential H-bridge, IN+/IN− complementary PWM |
| Sensor | Semitec 103JT-025 | R25 = 10 kΩ, B25/85 = 3435 K |
| Host link | USB CDC (Virtual COM) | 115200 baud (line coding ignored) |

## Pin map

| Pin | Function | Direction |
|---|---|---|
| PA0 | NTC voltage divider tap (ADC1_IN0) | analog in |
| PA8 | DRV592 IN+ (TIM1_CH1, 20 kHz PWM) | AF push-pull out |
| PA9 | DRV592 IN− (TIM1_CH2, 20 kHz PWM) | AF push-pull out |
| PB0 | DRV592 /SHUTDOWN (HIGH = enable) | GPIO out |
| PB1 | DRV592 HI-Z (HIGH = active drive) | GPIO out |
| PB10 | DRV592 /FAULT0 | GPIO in, pull-up |
| PB11 | DRV592 /FAULT1 | GPIO in, pull-up |
| PC13 | Status LED (active-low, blinks at 1 Hz) | GPIO out |
| PA11/PA12 | USB D−/D+ | (unchanged) |
| PA13/PA14 | SWDIO / SWCLK | (unchanged) |
| PD0/PD1 | HSE 8 MHz | (unchanged) |
| PC14/PC15 | LSE 32.768 kHz | (unchanged) |

### NTC voltage divider

```
       +3.3V
         │
       ┌─┴─┐
       │10k│   (1% metal-film top resistor)
       └─┬─┘
         ├─────────── PA0 (ADC1_IN0)
       ┌─┴─┐
       │NTC│   103JT-025
       └─┬─┘
         │
        GND
```

At 25 °C the divider sits at ~1.65 V; at 0 °C it rises to ~2.42 V.
Both well within ADC range. Add a 10 nF cap from PA0 to GND if your
wiring is long.

### DRV592 connections

- Connect IN+ ← PA8, IN− ← PA9.
- /SHUTDOWN ← PB0; HI-Z ← PB1. (Both must be HIGH for active drive.)
- /FAULT0 → PB10, /FAULT1 → PB11. The DRV592 outputs are open-drain;
  the MCU pull-ups handle pull-up.
- Wire the Peltier across OUT+ / OUT−.
- Power: VPVDD per DRV592 datasheet, with the recommended bulk and
  decoupling capacitors.

> **Heat / cool sign**: with the firmware's convention, **+duty drives
> current OUT+ → OUT−**. If that direction *cools* your Peltier when you
> wanted heat, just swap the Peltier leads at OUT+/OUT−.

## Clock tree (already configured)

- HSE 8 MHz → PLL ×6 → SYSCLK 48 MHz
- HCLK 48 MHz, APB2 48 MHz, APB1 24 MHz (timer clocks ×2 = 48 MHz)
- USB clock 48 MHz (PLL/1)
- ADC clock 12 MHz (PCLK2 /4) — corrected from /2 to stay within the F103's 14 MHz spec

## Real-time structure

```
TIM3 ─50 Hz─► HAL_TIM_PeriodElapsedCallback ─► Control_Tick50Hz
                                                  │
                                                  ├─► NTC_ReadTemperature()  (ADC poll, 8× avg)
                                                  ├─► Peltier_FaultActive()
                                                  ├─► PID_Update()
                                                  └─► Peltier_SetOutput()

main loop ─► CLI_Process()        (drains CDC RX ring, dispatches commands)
        └─► CLI_StreamTelemetry() (10 Hz "T=… SP=… U=…" line)
```

- TIM3 IRQ priority **1**, USB IRQ priority **0** → USB cannot be blocked
  by the control loop.
- Tick budget at 50 Hz: ADC 8× ~170 µs + math + PWM update ≪ 1 ms; main
  loop has ~19 ms of slack.

## Module overview

```
Core/Inc, Core/Src
├── ntc.{c,h}      — ADC1 calibration, polled read, β-equation conversion
├── pid.{c,h}      — discrete PID, d-on-measurement, conditional anti-windup
├── peltier.{c,h}  — DRV592 driver: signed duty → IN+/IN− complementary PWM
├── control.{c,h}  — 50 Hz loop glue, fault latch, setpoint clamp 0–25 °C
└── cli.{c,h}      — USB CDC ASCII command parser and 10 Hz telemetry
```

Defaults (all live-tunable over CLI):

- Setpoint: 25 °C, clamped to 0–25 °C
- PID gains: Kp = 20, Ki = 0.5, Kd = 0
- PID output limits: ±1000 (= ±100 % duty)
- TIM1 PWM frequency: 20 kHz, 2400 duty steps
- Output disabled at boot — must `START` over CLI to begin driving

Latched fault sources (output forced off until `CLR`):

- Sensor open / short / out of range (−40 .. +80 °C)
- DRV592 /FAULT0 or /FAULT1 asserted (over-current, thermal, UVLO)

## CLI (USB CDC, line-terminated by `\r` or `\n`)

| Command | Action |
|---|---|
| `SET <degC>` | Set target temperature (clamped 0..25) |
| `GET` | Reply with current temperature |
| `KP <v>` / `KI <v>` / `KD <v>` | Tune PID gain |
| `START` / `STOP` | Enable / disable output |
| `STATUS` | Full state dump |
| `STREAM ON` / `STREAM OFF` | Toggle 10 Hz telemetry |
| `CLR` | Clear latched fault |
| `HELP` or `?` | Print command list |

Telemetry stream format (10 Hz, on by default):
```
T=24.87 SP=25.00 U=-12 EN=1 FLT=0
```

`STATUS` example:
```
STATUS sp=10.00 t=18.32 duty=-742 en=1 flt=0 sens=1 Kp=20.000 Ki=0.500 Kd=0.000
```

## Build & flash

1. Open the project folder in **STM32CubeIDE 1.3.0**
   (`File → Open Projects from File System… → Peltier/`).
2. **Project → Build Project**. The IDE will pick up the new files in
   `Core/Src` automatically and regenerate `Debug/Core/Src/subdir.mk`.
3. Connect the board via SWD and **Run → Debug As → STM32 C/C++ Application**
   (or flash `Debug/Peltier.bin` with ST-LINK Utility / `STM32_Programmer_CLI`).
4. Plug the USB cable into the host. A new "USB Serial Device" (CDC ACM)
   should enumerate; open it in any terminal (PuTTY, picocom, screen,
   Tera Term, …). Send `HELP` to confirm communication.

> The `.ioc` was edited consistently with the C source, so re-running
> *Project → Generate Code* should be a no-op for these peripherals.
> All hand edits live in `USER CODE` regions (or in the same locations
> CubeMX generates), so regen is safe.

## First-light bring-up

1. With nothing connected to OUT+/OUT−, power the board and verify the
   PC13 LED blinks at 1 Hz — that's proof the 50 Hz loop is running.
2. Connect the NTC. `STATUS` should show a plausible `t=…` near room
   temperature.
3. Connect the DRV592 (with bench-supply current limit set conservatively),
   then the Peltier last.
4. Send `SET 25` then `START`. Confirm telemetry. Lower setpoint
   gradually (`SET 20`, `SET 15`, …) and watch the duty respond.
5. Trigger fault by briefly shorting /FAULT0 to GND — expect `FLT=1`,
   duty zero, output disabled. `CLR` then `START` to recover.

## Known sharp edges

- `STREAM` defaults to ON at boot; if the host isn't reading, the TX
  call will time-out (~20 ms) and silently drop bytes — no CPU stall.
- The PID gains are intentionally conservative; tune for your specific
  thermal mass, heatsink, and fan.
- Default startup is `STOP`. This is deliberate — we don't want to
  drive the Peltier on accidental power-on with no host present.
