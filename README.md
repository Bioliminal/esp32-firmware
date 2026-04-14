# esp32-firmware

Firmware for a single-sensor EMG fatigue-detection demo on an ESP32. The user flexes a muscle, the ESP32 detects fatigue by comparing a rolling average against a calibrated peak, and a small vibration motor buzzes with patterns proportional to the fatigue level.

## Sketches

| Sketch | Purpose |
|--------|---------|
| `emg_read/emg_read.ino`       | Minimal raw ADC reader — for tuning the sensor gain knob and checking signal quality on the Serial Plotter. |
| `emg_fatigue/emg_fatigue.ino` | Full demo: 5 s calibration → rolling-average fatigue detection → patterned haptic feedback → CSV serial output. |

- **MCU:** ESP32-WROOM-32UE (ESP32 Dev Module in Arduino IDE)
- **EMG signal pin:** GPIO34 (ADC1_CH6, input-only)
- **Motor control pin:** GPIO25 (LEDC PWM, 2 kHz, 8-bit)
- **ADC resolution:** 12 bits (0–4095)
- **Serial baud:** 115200
- **Sample period:** ~2 ms (~500 Hz)

## Hardware

- ESP32-WROOM-32UE dev board
- MyoWare 2.0 EMG sensor (VIN, GND, ENV pins soldered)
- Electrode cable + three snap electrodes (red / blue / black)
- DFRobot FIT0774 mini vibration motor
- 2N7000 N-channel MOSFET (low-side switch for the motor)
- 220 Ω gate resistor
- Jumper wires, breadboard

## Wiring

### EMG sensor

| MyoWare pin | ESP32 pin | Notes                          |
|-------------|-----------|--------------------------------|
| VIN (+)     | 3V3       | Power from ESP32 3.3 V rail    |
| GND (−)     | GND       | Common ground                  |
| ENV         | GPIO34    | Envelope output → ADC1_CH6     |

### Motor driver (low-side N-MOSFET)

| Connection                    | Notes                                   |
|-------------------------------|-----------------------------------------|
| ESP32 GPIO25 → 220 Ω → MOSFET **gate** | Current-limit + slight slew-rate damping |
| MOSFET **source** → GND       | Shared ground with ESP32                |
| MOSFET **drain** → motor (−)  | Motor switched on the low side          |
| 3V3 → motor (+)               | Motor always sees +3.3 V                |

> GPIO34 is input-only and has no internal pull-up/pull-down — ideal for analog input. GPIO25 supports LEDC PWM, which the firmware uses at 2 kHz / 8-bit for patterned haptics.

## Circuit diagram

```
  ┌──────────────────┐                      ┌──────────────────┐
  │   MyoWare 2.0    │                      │      ESP32       │
  │                  │                      │  (WROOM-32UE)    │
  │  VIN  ───────────┼──────────────────────┤ 3V3              │
  │  GND  ───────────┼───────────┬──────────┤ GND              │
  │  ENV  ───────────┼───────────┼──────────┤ GPIO34 (ADC1_CH6)│
  │                  │           │          │                  │
  └──┬──┬──┬─────────┘           │          │ GPIO25 ──┐       │
     │  │  │                     │          │          │       │
     │  │  └─ Reference (bony, e.g. elbow)  │          │       │
     │  └──── Mid-muscle electrode          │          │       │
     └─────── End-muscle electrode          │          │       │
                                            │          │       │
                                            │        [220 Ω]   │
                                            │          │       │
                                            │          ▼       │
                                            │      ┌───────┐   │
                                            │      │ 2N7000│   │
                                            │  G ──┤ gate  │   │
                                            │      │       │   │
                                            │  S ──┤source │   │
                                            │      │       │   │
                                            │  D ──┤ drain │   │
                                            │      └───┬───┘   │
                                            │          │       │
                                            │          │       │
                                            │    motor(−)      │
                                            │      ┌───┴───┐   │
                                            │      │  DC   │   │
                                            │      │ motor │   │
                                            │      └───┬───┘   │
                                            │    motor(+)      │
                                            │          │       │
                                            └──────────┼───────┘
                                                       │
                                                3V3 ───┘

                          │
                          ▼
                    USB ── Host PC (Serial Monitor @ 115200 baud)
```

MOSFET pinout (TO-92, flat side facing you): **1 = Source, 2 = Gate, 3 = Drain**.

## Build & flash

1. Open the sketch you want (`emg_read` or `emg_fatigue`) in Arduino IDE 2.x.
2. **Board:** ESP32 Dev Module. **Port:** `/dev/cu.usbserial-3110` (or whatever your board enumerates as).
3. Upload.
4. Open the Serial Monitor at **115200 baud**.

## Using `emg_fatigue`

1. Place electrodes on the target muscle (e.g. bicep) and reference on a nearby bony area.
2. Press the **EN** button on the ESP32 to start a session.
3. During the first 5 seconds, **flex as hard as you can** — the firmware prints `CAL:` lines and captures the peak.
4. After calibration you'll see a CSV header followed by rows of:
   ```
   timestamp_ms,avg,calibration_peak,fatigue_pct,motor_on
   ```
5. As you do reps and the envelope drops relative to the calibration peak, `fatigue_pct` rises and the motor pulses in progressively more urgent patterns:

   | Fatigue %  | Pattern                                      |
   |------------|----------------------------------------------|
   |   0–20 %   | Off                                          |
   |  20–40 %   | Slow pulse — 300 ms on / 700 ms off, 40 % PWM|
   |  40–60 %   | Medium pulse — 200 ms on / 300 ms off, 60 %  |
   |  60–80 %   | Fast pulse — 100 ms on / 100 ms off, 80 %    |
   |  80–100 %  | Near-continuous — 500 ms on / 50 ms off, 100 %|

### Capturing a session to CSV

```
cat /dev/cu.usbserial-3110 > session.csv
```

Press **EN** on the board to start, then Ctrl-C when done.

## Notes & known issues

- The gain knob on the MyoWare is physical — tune it so that a hard flex gives `avg` peaks around 500–2000 during the `CAL:` phase.
- Calibration only runs once per boot. Press **EN** to restart a session.
- Between-rep valleys currently count as "fatigue" — rep detection is a planned improvement.
- Habituation to a steady buzz is why the motor uses patterned PWM instead of continuous drive.
