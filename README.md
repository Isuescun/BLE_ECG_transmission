# BLE_sineWaves

Firmware and plotting tools for a wireless multi-channel ECG acquisition system. Streams 19 channels of 16-bit biosignal data at 1kHz over BLE from a XIAO nRF52840 peripheral to an ESP32 central, forwarded to a PC via USB serial.

## System architecture

```
Electrodes → SIgnal Acquisition → (SPI) → XIAO nRF52840 → (BLE) → ESP32 Central → (USB Serial) → PC / UI
```

> Note: The peripheral currently generates a fake sine wave per channel for pipeline testing.

## Repository structure

```
firmware/
├── peripheral_xiao.ino   — XIAO nRF52840 (Bluefruit): signal acquisition, BLE notify, battery + command handling
└── central_v5.ino        — ESP32 (Arduino BLE): BLE central, fragment reassembly, serial forwarding, command relay
plot/
└── plot_v5.py            — Python: serial decoder, live 19-channel plot, battery button
```

## Hardware

| Device | Role |
|---|---|
| XIAO nRF52840 Sense | Peripheral — acquires and streams data |
| ESP32 (any variant) | Central — relays BLE data to PC over serial |

## Getting started

### 1. Flash the firmware

Open each firmware folder as a PlatformIO project in VS Code. 
Or open each file in Arduino IDE

**Peripheral** (`platformio.ini`):
```ini
[env:xiaonrf52840]
platform = nordicnrf52
board = xiaonrf52840
framework = arduino
monitor_speed = 1000000
```

**Central** (`platformio.ini`):
```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
monitor_speed = 1000000
```

Upload peripheral first, then central. The XIAO blue LED will blink while advertising and go solid once the ESP32 connects.

### 2. Run the plot

Install dependencies:
```bash
pip install pyserial matplotlib numpy
```

Update the COM port in `plot_v5.py`:
```python
PORT = "COM8"   # Windows — check Device Manager for your port
                # Linux/Mac: "/dev/ttyUSB0" or "/dev/cu.usbserial-..."
```

Run:
```bash
python plot_v5.py
```

You should see 19 stacked channels updating live. Click **Get Battery** to request the XIAO's battery level on demand.

## Packet format

```
BLE notify (382 bytes):
  [packet_id_lo][packet_id_hi]          2 bytes — drop detection
  [t0_ch0_lo][t0_ch0_hi]               2 bytes — channel 0, time step 0
  ...
  [t9_ch18_lo][t9_ch18_hi]             2 bytes — channel 18, time step 9

Serial frame to PC:
  [0xAA][0x55]                          sync word
  [382 bytes packet]
  [0x0A]                                newline delimiter
```

10 time steps × 19 channels per BLE notify → 100 notifies/sec at 1kHz sample rate.

## Commands (bidirectional)

Send from PC terminal or plot UI → central forwards to XIAO over BLE:

| Command | Response | Description |
|---|---|---|
| `GET_BATTERY` | `BATTERY:87` | Battery percentage (requires LiPo connected) |
| `START` | `OK:START` | Start sampling (placeholder) |
| `STOP` | `OK:STOP` | Stop sampling (placeholder) |

## Status

- [x] BLE streaming — 19ch, 16-bit, 1kHz
- [x] Packet counter + drop detection
- [x] Battery level on demand
- [x] Bidirectional command pipeline
- [ ] INTAN SPI interface — pending hardware
- [ ] Impedance measurement — pending hardware
- [ ] UI integration
