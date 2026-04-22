# BMS_A123 BLE Prototype (STM32F302R8 + X-NUCLEO-BNRG2A1)

This repository now includes a **bare-metal BLE prototype** focused on your immediate goal:

- Advertise as `BMS-A123`
- Expose a custom BLE service for:
  - 13 cell voltages
  - 13 cell temperatures
- Stream updates every 1 second (notifications)
- Provide a browser dashboard using Web Bluetooth

## Hardware target

- MCU: **STM32F302R8**
- BLE expansion board: **X-NUCLEO-BNRG2A1** (BlueNRG-2 network coprocessor mode)
- SPI mode: **Mode 1** (`CPOL=0`, `CPHA=1`)
- SPI speed: configured around **1 MHz** from 8 MHz HSI

### Pin mapping used

| Signal | Arduino | STM32 pin |
|---|---|---|
| SPI SCK | D13 | PA5 |
| SPI MISO | D12 | PA6 |
| SPI MOSI | D11 | PA7 |
| BLE CS | A1 | PA1 |
| BLE IRQ | A0 | PA0 |
| BLE RESET | D7 | PA8 |

## BLE GATT layout

- Service UUID: `0xA123`
- Voltage characteristic UUID: `0xA124`
  - 26 bytes = 13 x `uint16` (little-endian), units: **mV**
- Temperature characteristic UUID: `0xA125`
  - 26 bytes = 13 x `int16` (little-endian), units: **0.1 degC**

## Files added

- `Inc/bms_ble.h`
- `Src/bms_ble.c`
- `web/dashboard.html`

`Src/main.c` now initializes BLE and runs the BLE process loop.

## Current behavior

At this stage, voltage/temperature values are generated from internal **stub data** (synthetic waveform).
This lets you fully validate BLE + browser integration before the SPI sensor subsystem is available.

## Browser dashboard

Open `web/dashboard.html` with Chrome or Edge (Web Bluetooth capable environment).

Notes:
- Web Bluetooth requires HTTPS (or localhost).
- iOS Safari does not support Web Bluetooth.

## Next step when sensor SPI details are ready

Replace the stub generator in:

- `ble_update_stub_measurements()` in `Src/bms_ble.c`

with values read from your measurement SPI peripheral, then keep the same payload packing.
