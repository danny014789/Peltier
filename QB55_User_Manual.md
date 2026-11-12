# QB55 Peltier Temperature Controller - User Manual

Hardware: ST NUCLEO-WB55RG (STM32WB55RG) - Firmware: `QB55_BLE_UART` - PC app: `QB55 BLE UART Monitor`
Document written 2026-11-12 from the project source and the test results of this project.

**How to read the labels in this manual**

| Label | Meaning |
|---|---|
| **[tested]** | I ran it on the real board and checked the result. |
| **[source]** | Taken from the code; not separately run. |
| **[assumed]** | A choice made without data from your hardware. You must confirm it. |
| **[not tested]** | Needs hardware that has not been connected yet (NTC, DRV595, Peltier). |

---

## 1. What this system does

You type a temperature on a Windows PC. The PC sends it to the Nucleo board over Bluetooth Low Energy.
The board reads a 10 k NTC thermistor, runs a PID loop at 10 Hz, and produces a small control voltage on
**IN+**. A DRV595 power amplifier turns that voltage into drive for a Peltier element, which cools the
thing the thermistor is attached to.

```
 Windows PC                 NUCLEO-WB55RG                         your hardware (not yet built)
+-----------+   Bluetooth  +---------------------+
| QB55 app  | <==========> |  PID loop, 10 Hz    |  IN+  (PA0) --[RC filter]--> DRV595 IN+ --+
| (or CLI)  |   LE, text   |                     |  IN-  (PA1) --[RC filter]--> DRV595 IN-   +--> Peltier
+-----------+   commands   |                     |                                           |
                           |  ADC  <--- PC3 <--- NTC divider (3V3 - 10k - node - NTC - GND) |
                           +---------------------+
```

The Nucleo has **no DAC**. The "analog" outputs IN+ and IN- are PWM signals, and each needs an RC low-pass
filter outside the board to become a steady voltage (section 5.1).

---

## 2. Where everything is

All paths are under `C:\Users\ASUS\STM32CubeIDE\workspace_2.2.0\`.

| What | Path |
|---|---|
| This manual | `QB55_User_Manual.md` |
| PC app, ready to run | `QB55_BLE_Monitor\dist\QB55_BLE_Monitor.exe` |
| PC app source / console mode | `QB55_BLE_Monitor\qb55_ble_monitor.py` (needs Python + `bleak`) |
| Firmware project | `QB55_BLE_UART\` |
| Built firmware | `QB55_BLE_UART\Release\QB55_BLE_UART.elf` (also `.hex`, `.bin`) |
| Temperature controller code | `QB55_BLE_UART\Core\Src\qb55_thermo.c`, `Core\Inc\qb55_thermo.h` |
| IN+/IN- output code | `QB55_BLE_UART\Core\Src\qb55_in_out.c`, `Core\Inc\qb55_in_out.h` |
| Command dispatch (LED, IN+, temperature) | `QB55_BLE_UART\STM32_WPAN\App\p2p_server_app.c` |

There is **no `.ioc` file**, so CubeMX cannot be used on this project. Edit the source directly (section 14).

---

## 3. Pinout

### 3.1 Pins used by this firmware

| Signal | MCU pin | Header position (from ST's examples) | Type | Notes |
|---|---|---|---|---|
| **IN+** | **PA0** (TIM2 CH1) | **CN7 pin 34** | PWM output | 0 V at boot. Needs an RC filter. |
| **IN-** | **PA1** (TIM2 CH2) | **CN7 pin 32** (Arduino CN8 pin A2) | PWM output | 0 V at boot. Manual only (not PID). |
| **NTC** | **PC3** (ADC1 channel 4) | **CN7 pin 36** (Arduino CN8 pin A4) | Analog input | Midpoint of the thermistor divider. |
| 3V3 | - | the pin labelled 3V3 on the Nucleo power header | Supply | Divider top. **Same rail as the ADC reference.** |
| GND | - | any GND pin | Ground | Common ground for filter, NTC, DRV595 signal. |

Header positions for PA0, PA1 and PC3 come from ST's own example readmes for this board **[source]**. I did
not find ST's numbers for the 3V3 and GND header pins in this project, so use the labelled pins and check
them against the Nucleo user manual (UM2435) before wiring.

### 3.2 Fixed on the board (for reference)

| Item | MCU pin | Used by this firmware? |
|---|---|---|
| LD1, blue LED | PB5 | Not used. **Does not light on this unit.** |
| LD2, green LED | PB0 | ST's code: flickers on advertising and radio activity (section 11). |
| LD3, red LED | PB1 | **The `led on` / `led off` command.** |
| B1 / SW1 button | PC4 | ST's demo code: sends 2 raw bytes to the PC (section 11). |
| B2 / SW2 button | PD0 | ST code present; no function in this firmware. |
| B3 / SW3 button | PD1 | ST code present; no function in this firmware. |
| RESET button | NRST | Restarts the board. Restores all defaults. |
| ST-Link serial port | PB6 (TX), PB7 (RX), USART1, 115200 | Configured for ST's debug traces **[source; output not checked]**. |
| SWD programming | PA13 (SWDIO), PA14 (SWCLK), PB3 (SWO) | Used by the ST-Link for flashing. Do not use these pins. |
| USB user port | PA11 / PA12 | Not used (USB was removed from the project). |

### 3.3 Pins to avoid reusing

PA0, PA1 and PC3 are taken by this firmware. PA13, PA14 and PB3 belong to programming. PB6/PB7 belong to the
ST-Link serial port.

---

## 4. Specifications

| Item | Value |
|---|---|
| Bluetooth name | `QB55_UART` (address on this unit: `00:80:E1:27:98:6E`) |
| Bluetooth service | Nordic UART Service, `6E400001-B5A3-F393-E0A9-E50E24DCCA9E` |
| Board -> PC (notify) | `6E400003-B5A3-F393-E0A9-E50E24DCCA9E` |
| PC -> board (write) | `6E400002-B5A3-F393-E0A9-E50E24DCCA9E`, max 20 bytes per write |
| Line length | Up to 64 characters per command line; line ends at LF (CR is ignored) |
| Wireless stack on the chip | BLE full stack 1.24.0 with FUS 2.2.0 (must stay matched to the firmware) |
| Control rate | 10 Hz (100 ms) |
| Temperature input | PC3, 12-bit ADC, 16x hardware averaging, then 0.3 smoothing per tick |
| Output PWM | TIM2, 10-bit, 31.25 kHz, 3.2 mV per step |
| Output full scale | 3.3 V (the firmware assumes exactly 3.3 V; real supply may read about 3.25 V) **[assumed]** |
| PID output ceiling | 1.000 V, fixed in firmware (`pid max` can lower it, never raise it) |
| Setpoint range | 0 to 30 C |
| Over-temperature cutoff | 85 C measured |
| Sensor fault | Reading within 100 counts of either ADC end, or outside -40 to 150 C, for 5 ticks (0.5 s) |
| Program size | about 63 KB of the 1 MB flash |

---

## 5. Wiring guide (hardware not yet built, so none of this is tested)

### 5.1 IN+ and IN- filter, one per pin

The recommended single-stage filter for the parts you have is a **100 kOhm resistor and a 104 ceramic
capacitor (100 nF)**, with the firmware set to 10-bit PWM (already the case).

```
 PA0 (IN+) ----[100k]----+---- to DRV595 IN+
                          |
                       104 ceramic (100 nF)
                          |
                         GND
 PA1 (IN-) ----[100k]----+---- to DRV595 IN-     (same again)
                          |
                       104 ceramic
                          |
                         GND
```

* Use ceramic or film capacitors. **Do not use the aluminium electrolytic**: its leakage current across
  100 k shifts the voltage by tens of millivolts, unpredictably.
* Expected with this filter (calculated, not measured): about 2.6 mV ripple, about 70 ms to settle.
* **The 100 k resistor is the source impedance.** Whatever it feeds must have an input resistance of about
  **10 MOhm or more** to stay within 1 percent. **I have not seen the DRV595 datasheet.** Before connecting,
  check its input resistance and input bias requirements. If it is lower than about 10 MOhm, put an op-amp
  buffer (voltage follower) between the filter and the DRV595, or accept that the voltage at the DRV595
  will be lower than the number you typed. The PID still closes the loop on temperature either way.
* A two-stage filter (two resistors and two capacitors per pin) gives far less ripple, and the firmware can
  go back to 12-bit with it. See `QB55_PWM_BITS` in `qb55_in_out.h`.

### 5.2 Thermistor divider

```
 3V3 ----[ 10k fixed ]----+----[ NTC 10k, 103JT-050 ]---- GND
                          |
                          +---- to PC3
                          |
                       100 nF to GND (recommended, close to the board)
```

* The divider must be fed from the board's **3V3**. The firmware computes temperature from the *ratio* of
  the ADC reading to full scale, which is only correct if the divider and the ADC share the same supply. Never
  feed it from more than 3.3 V.
* **Put the fixed 10 k resistor on the board side** of any cable. If the cable breaks, PC3 is then pulled to
  3V3, which the firmware detects as an open sensor. An unconnected PC3 reads as a believable
  temperature that drifts (48 to 70 C seen on this board with nothing attached) and **cannot be
  detected**.
* The firmware assumes the NTC is on the **ground side**, as drawn **[assumed]**. If yours is on the 3V3
  side, set `QB55_NTC_ON_LOW_SIDE` to 0 in `qb55_thermo.h` and rebuild.
* The firmware assumes B = 3435 K and R25 = 10 kOhm **[assumed]**. Check your 103JT-050 datasheet and change
  `QB55_NTC_BETA_K` if it differs. This number sets the accuracy.

### 5.3 DRV595 and Peltier

You told me the DRV595 has about 26 dB of gain (x20), so a 1 V input can request up to about 20 V at the
output if the DRV595 supply allows it. Check before connecting:

1. **Peltier ratings.** Make sure 1.000 V x gain (limited by the DRV595 supply) is not above your Peltier's
   rated voltage and current. If it is, lower the ceiling with `pid max` or change `QB55_PID_OUT_MAX_MV`.
2. **Direction.** The firmware starts in **cooling** mode: it raises IN+ when the temperature is above the
   setpoint. If a positive IN+ turns out to *warm* the cold plate, swap the two Peltier wires. (Heat mode
   cannot cool below room temperature.)
3. **Hot side.** The single thermistor sees only the cold side. If the hot side has no heat sink or fan it can
   overheat, and **the 85 C cutoff will not protect it**. Fit a heat sink and fan; a second thermistor with
   its own cutoff is the proper fix (not built).
4. **Grounding.** Keep the Peltier/DRV595 power return current away from the signal ground. Join the two
   grounds at one point.

---

## 6. First start

1. Plug the Nucleo into the PC by USB (the ST-Link port).
2. The firmware starts immediately. It advertises as `QB55_UART`. **After 60 seconds without a connection it
   stops advertising.** Press the black **RESET** button to start another 60-second window.
3. Start `QB55_BLE_Monitor.exe`. It scans for 5 seconds, lists the devices it hears, and pre-selects
   `QB55_UART` if present.
4. Click **Connect**.

State after every reset **[tested]**: PID off, IN+ and IN- at 0 V, mode cool, no setpoint, Kp 0.10,
Ki 0.005, Kd 0, ceiling 1.000 V, log off, simulator off, LD3 off.

---

## 7. The Windows app

### 7.1 The window

| Control | What it does |
|---|---|
| **Device** list | Devices heard in the last scan, strongest signal first, as `name [address] rssi`. |
| **Scan** | Listens for 5 seconds and refills the list. |
| **Connect / Disconnect** | Connects to the selected device (waits up to 20 s to hear it). |
| Status dot and text | Grey idle, amber working, green connected. |
| **Show unnamed devices** | Most nearby devices have no name and are hidden; tick to see them. |
| Log area | Grey timestamps; black text from the board; green `>` lines you sent; blue notes; red errors. |
| **Send** box, **Ends with**, **Send** | Type a command and press Enter. Ends with: LF (default), CR+LF or None. Enabled only while connected. |
| **Hex view** | Shows each received packet as hex plus text. |
| **Auto-scroll**, **Clear** | Keep the log scrolled to the bottom; empty the log. |
| Counters | Lines/bytes received and bytes sent. |

Every line you send is echoed by the board as `echo: <your line>`, followed by the board's reply. If you send
several commands at once (for example with several `--send` options), a reply can appear *after the echo of
the next command*. The content is correct; only the order on screen is shifted. I saw this on the real board
today; typing one command at a time is the normal use.

**If `QB55_UART` is not in the list:** press RESET on the board and click **Scan**. After disconnecting, wait
about 3 seconds before reconnecting, because Windows keeps the link open briefly.

### 7.2 Console mode (for scripts and testing)

```
python qb55_ble_monitor.py --cli --scan                          list nearby devices
python qb55_ble_monitor.py --cli --seconds 10 --send "status"    connect, send, print replies, exit
python qb55_ble_monitor.py --cli --seconds 60 --send "log on"    capture a minute of the 1 Hz log
```

| Option | Meaning |
|---|---|
| `--cli` | No window; print to the console. |
| `--scan` | List nearby devices and exit. |
| `--send TEXT` | Send this line after connecting (repeat the option for several lines). |
| `--eol LF/CR+LF/None` | Line ending added to `--send` text (default LF). |
| `--seconds N` | Stop after N seconds. Exit code 0 only if something was received. |
| `--name PREFIX`, `--address ADDR` | Choose the device by name prefix (default `QB55`) or exact address. |
| `--hex`, `--out FILE` | Show packets as hex; also write the output to a file. |
| `--autoconnect` | Window mode: connect by itself after the first scan. |

---

## 8. Command reference

Type these in the Send box. **Case does not matter**; extra spaces around a command are ignored. Replies are
shown below as sent by the board.

### 8.1 Temperature control

| Command | Reply / effect |
|---|---|
| `set 15` or `temp 15` | `SP = 15.0 C`. Range 0 to 30. A trailing `c` is accepted (`set 15c`). |
| `temp` or `set` alone | `T = 24.87 C`, the measured temperature. |
| `pid on` | Starts control: `PID on: cool, T=25.0C -> SP=15.0C, output max 1.000 V`. |
| `pid off` | `PID off, IN+ = 0 V`. Output goes to 0 V and the PID stops. |
| `pid mode cool` / `pid mode heat` | Direction of the drive. Only allowed while the PID is off. |
| `pid kp 0.2`, `pid ki 0.01`, `pid kd 0.5` | Set a gain. With no value, report it. Ranges: kp and ki 0 to 100, kd 0 to 1000. |
| `pid max 0.8` | Lower the output ceiling (0 to 1.0 V). A trailing `v` is accepted. |
| `status` or `pid` | Two lines: temperature, setpoint, output, state, mode, then gains, ceiling, source, ADC count, thermistor resistance. |
| `log on` / `log off` | While on, one line a second: `T=24.87 SP=15.00 OUT=0.412V RUN`. Default off. |
| `help` | One-line command list. |

Units: Kp in volts per degree, Ki in volts per degree-second, Kd in volt-seconds per degree.

Common errors **[tested]**:

| You see | Meaning |
|---|---|
| `error: set a temperature first, e.g.  set 40` | `pid on` needs a setpoint first. |
| `error: setpoint must be 0.0 to 30.0 C` | Out of range or not a number. |
| `error: no valid temperature reading (sensor open/short? ...)` | The thermistor reading is unusable. |
| `error: send  pid off  before changing the mode` | Mode can only change while off. |
| `error: kp must be a number from 0 to 100.000` | Gain out of range. |
| `error: max must be a number from 0 to 1.000` | Ceiling above 1.0 V is refused. |

Messages the board sends on its own **[tested]**:

| Message | Meaning |
|---|---|
| `FAULT: no valid temperature (sensor open/short?). PID stopped, IN+ = 0 V` | Sensor open, shorted or implausible for 0.5 s. |
| `FAULT: over-temperature cutoff. PID stopped, IN+ = 0 V` | Measured above 85 C. |

After either fault the output is 0 V and stays off until you send `pid on` again.

### 8.2 Direct outputs

| Command | Reply / effect |
|---|---|
| `IN+ 2V` | `IN+ = 2.001 V`. Sets IN+ to about 2 V. Accepted forms: `IN+ 2`, `IN+ 2000mV`, `IN+ 2 V`, `in+2v`. |
| `IN- 1.5V` | Same for IN-. |
| `IN+` / `IN-` alone | Report the present level. |
| `IN+ 3.4V` | `error: IN+ max is 3.300 V` (output unchanged). |

* The reply shows the value the PWM **actually produced**, rounded to its 3.2 mV step (so `2V` reads
  `2.001 V`).
* Values stay after you disconnect. Only a reset returns them to 0 V.
* **While the PID is running, `IN+` is refused:** `error: IN+ is under PID control; send  pid off  first`.
  `IN-` stays manual.

### 8.3 LED

| Command | Effect |
|---|---|
| `led on` / `led off` | Switch the red LED, LD3. Reply `LED is ON` / `LED is OFF`. |

### 8.4 Simulator (for testing without hardware)

| Command | Effect |
|---|---|
| `sim on` | Replace the thermistor with a made-up thermal plant that starts at 25 C. |
| `sim off` | Back to the real thermistor (also stops the PID if it was running). |
| `sim open` / `sim short` / `sim ok` | While on: pretend the thermistor is broken, or repaired. |
| `sim temp 140` | While on: jump the simulated temperature (to test the cutoff). |

The simulated plant is a simple first-order model (ambient 25 C, 30 C per volt, 10 s time constant) and is
**not your hardware**. It feeds a raw ADC count through the same conversion and fault checks as the real
sensor, so the thermistor maths and fault handling are exercised, but gains tuned on it will not carry over.

---

## 9. Using the temperature controller

### 9.1 How the PID works **[tested with the simulator]**

* Error is "how far on the wrong side of the setpoint". In cool mode the error is positive when it is too
  warm.
* Output = Kp x error + integral + derivative, limited to 0 up to the ceiling (at most 1.000 V). It can
  never go negative, so control is **one-sided**: in cool mode, warming up again relies on room temperature.
* The integral stops growing while the output is already at the ceiling and still wants more
  (anti-windup), so an unreachable setpoint does not leave a large built-up value behind.
* The derivative acts on the measurement, not the error, so changing the setpoint causes no kick.
* The reading is smoothed (about 0.3 s) before it is used.

### 9.2 Bring-up checklist with real hardware **[not tested]**

Do these in order. Keep the Peltier unpowered for steps 1 to 3.

1. **Disconnect the DRV595 input.** Direct `IN+` commands can reach 3.3 V, which would be about 66 V at the
   output with a x20 amplifier.
2. **Check the outputs.** Connect a multimeter to the filter output. Send `IN+ 1V`, `IN+ 2V`. Compare.
3. **Find your real supply voltage.** Send `IN+ 3.3V` and measure. That reading is the real VDD (the board
   measured about 3.25 V). Tell the firmware by editing `QB55_VDD_MV` in `qb55_in_out.h` and rebuilding, and
   the other voltages will then be closer. Send `IN+ 0V` afterwards.
4. **Check the thermistor.** Send `temp` and compare with a reference thermometer. If it is off by a
   consistent amount, check the B value and divider orientation (section 5.2).
5. **Reconnect the DRV595 and Peltier with a low ceiling:** `pid max 0.2`.
6. **Check the direction.** `set 20`, `pid on`, `log on`. The temperature should fall. If it rises, send
   `pid off` at once and swap the Peltier wires.
7. Raise the ceiling gradually (`pid max 0.5`, then 1.0) while watching the Peltier hot side.

### 9.3 Tuning

Defaults are conservative placeholders: Kp 0.10 V/C, Ki 0.005 V/(C s), Kd 0. A general approach, not yet
tried on your hardware:

1. Set `pid ki 0` and `pid kd 0`. With `log on`, raise Kp until the response is fast without oscillating.
2. Add Ki slowly to remove the last offset. Too much Ki causes overshoot and slow oscillation.
3. Add a little Kd only if there is overshoot you cannot remove otherwise.
4. Write down the final gains. **They are lost at reset** and must be typed again (or changed in the source).

### 9.4 Recovering from a fault

1. Find and fix the cause: a thermistor wire, a short, or a heat-sink problem.
2. Check with `temp` that the reading is sensible.
3. Send `pid on` again.

---

## 10. Safety notes

* **Disconnect the DRV595 before any direct `IN+` test above 1 V.** The 1 V limit applies only to the PID
  output. Direct `IN+` commands go up to 3.3 V.
* **Confirm the Peltier's voltage and current ratings** against 1.000 V x the DRV595 gain and its supply.
* **Provide hot-side cooling.** The only temperature sensor is on the cold side.
* An unconnected thermistor input looks like a real temperature. Wire the fixed resistor on the board side.
* The 85 C cutoff is much higher than the 0 to 30 C setpoint range, so it only catches a serious failure. It
  can be lowered in `qb55_thermo.h` (`QB55_CUTOFF_C`); I suggested about 45 to 50 C.
* The controller keeps running if Bluetooth drops. A reset (RESET button or power loss) returns to PID off
  and 0 V.
* Cold plates can collect condensation.

---

## 11. LEDs and buttons

| Item | Behaviour |
|---|---|
| LD3 (red) | On/off with `led on` / `led off`. Off at boot. |
| LD2 (green) | Controlled by ST's code: lit around advertising, flickers with radio activity. |
| LD1 (blue) | Does not light on this unit. |
| B1 (SW1) | Sends 2 raw bytes (not text) on the TX characteristic and toggles the second byte each press. They show as odd characters in the log. Harmless. |
| B2, B3 | No function in this firmware. |
| RESET | Restarts the firmware and restores all defaults. |

---

## 12. Troubleshooting

| Symptom | Likely cause and fix |
|---|---|
| `QB55_UART` not in the device list | It stops advertising 60 s after boot or disconnect. Press RESET, click Scan. |
| Connect fails right after a disconnect | Windows holds the link for about 3 s. Wait and retry. |
| Send box is greyed out | Not connected. |
| `error: no valid temperature reading` | Thermistor open or shorted, or PC3 not wired. For a bench test of the software, use `sim on`. |
| `pid on` works with nothing connected | PC3 floats and reads a plausible temperature. Normal without wiring; not safe with a real Peltier. |
| Temperature reads wrong by a steady amount | Wrong B value, divider orientation, or the divider is not on the board's 3V3. |
| Voltage at the filter is a little off | The firmware assumes 3.300 V supply (section 9.2, step 3). |
| Voltage at the DRV595 is lower than typed | The DRV595 input loads the 100 k filter (section 5.1). |
| Temperature moves the wrong way | Peltier polarity. Swap the wires. |
| Output stuck at the ceiling for a long time | Setpoint unreachable, wrong direction, or no hot-side cooling. |
| Nothing at all from the board | Check USB power and that LD1/LD2 are not the only lights you expect; press RESET. |
| App says Bluetooth scan failed | Bluetooth is off in Windows. |

---

## 13. Known limitations

* **Not tested with real hardware:** the NTC, the DRV595 and the Peltier. Everything on the controller was
  tested with the simulator and with the real ADC unconnected.
* The thermistor B value (3435 K), divider orientation and the DRV595 behaviour are assumptions.
* Setpoint, gains and mode are not saved across a reset.
* No auto-tune. No warning for "output at maximum for too long". No second thermistor for hot-side cutoff.
* One Bluetooth client at a time in practice.
* The direct `IN+` command is not limited to 1 V (only the PID output is).
* I could not check what ST's debug traces print on the ST-Link serial port.

---

## 14. Maintenance

### 14.1 Rebuild the firmware

Use `make` directly (the Eclipse command-line build hangs afterwards). In `QB55_BLE_UART\Release`, with the
`make` and `arm-none-eabi` tools from the CubeIDE 2.2.0 plugins folder on the PATH:

```
C:\ST\STM32CubeIDE_2.2.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.make.win32_2.2.200.202604021615\tools\bin\make.exe all
```

(add `...\com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.14.3.rel1.win32_1.0.100.202602081740\tools\bin`
to the PATH first). Adding a new `.c` file requires listing it in `Release\Core\Src\subdir.mk` and
`Release\objects.list`.

### 14.2 Flash

```
"D:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe" ^
  -c port=SWD mode=UR -d C:\Users\ASUS\STM32CubeIDE\workspace_2.2.0\QB55_BLE_UART\Release\QB55_BLE_UART.elf -v -rst
```

The programmer's read size argument (`-r32 <address> <size>`) is in **bytes**.

### 14.3 Settings you may want to change (rebuild needed)

| Setting | File | Default |
|---|---|---|
| Thermistor B value | `qb55_thermo.h` `QB55_NTC_BETA_K` | 3435 |
| Divider orientation | `qb55_thermo.h` `QB55_NTC_ON_LOW_SIDE` | 1 (NTC to ground) |
| Fixed resistor | `qb55_thermo.h` `QB55_NTC_FIXED_OHM` | 10000 |
| Setpoint range | `qb55_thermo.h` `QB55_SET_MIN_C` / `QB55_SET_MAX_C` | 0 / 30 |
| Cutoff | `qb55_thermo.h` `QB55_CUTOFF_C` | 85 |
| Output ceiling | `qb55_thermo.h` `QB55_PID_OUT_MAX_MV` | 1000 |
| Default direction | `qb55_thermo.h` `QB55_PID_DEFAULT_HEAT` | 0 (cool) |
| Default gains | `qb55_thermo.c`, `QB55_Thermo_Init()` | 0.10 / 0.005 / 0 |
| PWM resolution | `qb55_in_out.h` `QB55_PWM_BITS` | 10 |
| Supply assumed for the outputs | `qb55_in_out.h` `QB55_VDD_MV` | 3300 |

### 14.4 If the wireless processor loses its stack

The firmware needs the BLE full stack 1.24.0 on the second core (CPU2). If Bluetooth never starts after
flashing, the stack may be missing or mismatched. Use the standalone STM32CubeProgrammer 2.14 (the copies
inside CubeIDE lack the FUS helper files). The order that worked on this board:
`-startfus`, `-fwdelete`, `-fwupgrade stm32wbxx_FUS_fw.bin 0x080EE000 firstinstall=0`, `-startfus` again,
`-fwupgrade stm32wb5x_BLE_Stack_full_fw.bin 0x080D0000 firstinstall=0`, then `-startwirelessstack` and reflash
the application. The binaries are in
`C:\Users\ASUS\STM32Cube\Repository\STM32Cube_FW_WB_V1.24.0\Projects\STM32WB_Copro_Wireless_Binaries\STM32WB5x\`.
Each of these erases the application flash, so reflash it afterwards.

---

## 15. Command cheat sheet

```
set 15            target temperature (0-30 C)       temp          read the temperature
pid on / off      start / stop control              pid mode cool|heat   direction (off only)
pid kp|ki|kd <x>  gains                             pid max <V>   output ceiling (<= 1.0)
status            full summary                      log on|off    1 line per second
IN+ <V>  IN- <V>  set an output (0-3.3 V)           IN+  IN-      read an output
led on|off        red LED LD3                       help
sim on|off        test without hardware             sim open|short|ok|temp <C>
```
