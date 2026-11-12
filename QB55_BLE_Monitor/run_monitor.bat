@echo off
rem Opens the QB55 BLE UART monitor without a console window (needs Python + bleak).
start "" pythonw "%~dp0qb55_ble_monitor.py" %*
