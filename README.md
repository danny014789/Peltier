# QB55 Peltier temperature controller

NUCLEO-WB55RG firmware (BLE UART, PWM outputs IN+/IN-, PID temperature control with a 10 k NTC)
and a Windows BLE monitor app.

- `QB55_BLE_UART/` - STM32CubeIDE 2.2.0 project (no .ioc; build with `make` in `Release/`)
- `QB55_BLE_Monitor/` - Python/bleak GUI and console app (`pip install -r requirements.txt`)
- `QB55_User_Manual.md` - pinout, wiring, commands, tuning, build and flash

Needs BLE full stack 1.24.0 and FUS 2.2.0 on the wireless core (see the manual, section 14).
