# esp32-firmware

Firmware for reading an EMG (electromyography) sensor with an ESP32.

## Overview

`emg_read/emg_read.ino` samples an analog EMG signal on GPIO34 using the ESP32's 12-bit ADC and streams the raw values over the USB serial port at 115200 baud. The loop delays 2 ms between samples, giving an approximate sample rate of ~500 Hz.

- **MCU:** ESP32
- **Signal pin:** GPIO34 (ADC1_CH6, input-only)
- **ADC resolution:** 12 bits (0–4095)
- **Serial baud:** 115200
- **Sample period:** ~2 ms

## Hardware

You need an ESP32 dev board, an EMG front-end module (e.g. MyoWare 2.0 or equivalent), three surface electrodes, and jumper wires.

### Wiring

| EMG sensor pin | ESP32 pin  | Notes                          |
|----------------|------------|--------------------------------|
| VCC (+)        | 3V3        | Power from ESP32 3.3 V rail    |
| GND (−)        | GND        | Common ground                  |
| SIG / OUT      | GPIO34     | Analog signal into ADC1_CH6    |

> GPIO34 is input-only and has no internal pull-up/pull-down — perfect for an analog input. Keep the sensor lead short to reduce noise.

### Circuit diagram

```
          ┌─────────────────┐                  ┌──────────────────┐
          │   EMG Sensor    │                  │      ESP32       │
          │   (MyoWare)     │                  │                  │
          │                 │                  │                  │
  RED ────┤ +  (VCC)        ├──────────────────┤ 3V3              │
          │                 │                  │                  │
  BLK ────┤ −  (GND)        ├──────────────────┤ GND              │
          │                 │                  │                  │
          │    SIG / OUT    ├──────────────────┤ GPIO34 (ADC1_CH6)│
          │                 │                  │                  │
          └────┬──┬──┬──────┘                  └──────────────────┘
               │  │  │
               │  │  └── Reference electrode (bony area, e.g. elbow)
               │  └───── Mid-muscle electrode
               └──────── End-muscle electrode

                        │
                        ▼
                  USB ── Host PC (Serial Monitor @ 115200 baud)
```

## Build & flash

1. Open `emg_read/emg_read.ino` in the Arduino IDE (or use `arduino-cli`).
2. Select your ESP32 board and the correct serial port.
3. Upload the sketch.
4. Open the Serial Monitor at **115200 baud** to see the stream of raw ADC values.

## Output format

One integer per line, in the range `0`–`4095`:

```
1843
1851
1847
...
```

At rest you should see values hovering near the sensor's baseline; contracting the target muscle produces visible spikes.
