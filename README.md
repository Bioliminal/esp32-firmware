# esp32-firmware

Firmware for the Bioliminal closed-loop biofeedback garment. The ESP32 streams muscle-activity data from a MyoWare 2.0 EMG sensor and drives a small vibration motor for haptic cueing. The mobile app owns session logic, rep detection (camera + BlazePose), and the fatigue algorithm; this firmware is intentionally dumb — it captures raw signals and executes haptic commands.

Full architecture and the phase-by-phase plan live at `../notes/` (see [related docs](#related-docs)).

## Sketches

| Phase | Sketch | Purpose |
|---|---|---|
| 0 — legacy | `emg_read/emg_read.ino` | Minimal raw ADC reader for tuning the MyoWare gain pot and visualizing the signal in Serial Plotter. |
| 0 — legacy | `emg_fatigue/emg_fatigue.ino` | Original single-sketch demo: 5 s calibration → rolling-average fatigue detection → patterned haptic. Superseded by Phase 2 but kept as reference. |
| 0 — legacy | `motor_test/motor_test.ino` | Verifies the motor + MOSFET wiring (PWM ramp, fatigue-style pulse patterns). |
| 0 — legacy | `esp32_ble_test/esp32_ble_test.ino` | Minimal BLE peripheral matching the Bioliminal Garment app's GATT contract. Synthetic sine data on FF02 at 20 Hz for app-side development. |
| 0 — legacy | `emg_three_channel_test/emg_three_channel_test.ino` | No-BLE sanity check for RAW + RECT + ENV pins via Serial Plotter at 200 Hz. |
| 0 — legacy | `bicep_raw_stream/bicep_raw_stream.ino` | BLE-only raw streamer: 3 channels × 2 kHz → 308-byte packets at 40 Hz on FF02. No haptic, no commands. |
| 1 — data capture | `emg_button_capture/emg_button_capture.ino` | **Button-triggered data logger.** Short-press BOOT to start session and mark reps; long-press to end. Streams 2 kHz CSV over Serial at 921600 baud. Pairs with `../captures/capture.sh`. |
| 2 — realtime cueing | `bicep_realtime/bicep_realtime.ino` | **BLE production sketch.** Raw stream on FF02 + new FF04 command characteristic (`PULSE_BURST`, `STOP_HAPTIC`, `SET_SESSION_STATE`). Non-blocking pulse-burst scheduler supporting up to 4 motors. Auto-stops motors on BLE disconnect. |
| 2 — realtime cueing | `bicep_hardware_test/bicep_hardware_test.ino` | **No-BLE bench twin of `bicep_realtime`.** Same pulse scheduler, triggered via BOOT button (FADE/URGENT) or Serial commands (`fade`/`urgent`/`form`/`pulse <args>`/`stop`). Lets you validate the motor + EMG pipeline without the phone. |
| 3 — autonomous (hardware-led) | `bicep_autonomous/bicep_autonomous.ino` | **Hardware-led demo sketch.** Phone writes only `SET_SESSION_STATE(Calibrating)` / `SET_SESSION_STATE(Idle)`; firmware runs its own 5 s calibration, auto-advances to Active, counts reps locally from the ENV channel, and fires haptic cues autonomously on fatigue thresholds. Extends FF02 to **310 B** with `rep_count` + `cue_event` in the header. Pair with the app's `feat/hardware-led-read-only` branch. |

## Quick start

- **Bench-test the motor + EMG without the app:** flash `bicep_hardware_test`. Open Serial Monitor at 115200, type `fade`, motor buzzes.
- **Record an EMG dataset for offline analysis:** flash `emg_button_capture`, run `../captures/capture.sh <subject>`.
- **Run the full Bioliminal Garment paired with the app:** flash `bicep_realtime`. See [`../notes/haptic-cueing-handshake.md`](../notes/haptic-cueing-handshake.md) for the BLE protocol and Phase 2 scope.
- **Run the hardware-led demo:** flash `bicep_autonomous`. Pair the mobile app on `feat/hardware-led-read-only`. Phone only writes session boundaries; firmware owns rep counting, fatigue, and cue firing. See [`../notes/ble-flow.md`](../notes/ble-flow.md) for the autonomous-mode sequence.

## Hardware

- ESP32-WROOM-32UE dev board
- MyoWare 2.0 EMG sensor — **RAW, RECT, and ENV pins all soldered** (Phase 2 wiring)
- Electrode cable + three snap electrodes (reference on a bony landmark; two on the target muscle belly)
- DFRobot FIT0774 mini vibration motor (10 × 2.7 mm coin ERM, 3 V / 90 mA, ~200 Hz mechanical resonance)
- 2N7000 N-channel MOSFET (low-side switch) + 220 Ω gate resistor

### Key constants

- **MCU:** ESP32-WROOM-32UE (Arduino board: *ESP32 Dev Module*)
- **ADC resolution:** 12 bits (0–4095)
- **Sample rate (Phase 2 sketches):** 2 kHz (500 µs period) on all three MyoWare channels
- **Serial baud:** sketch-dependent — see the sketches table. `emg_button_capture` uses **921600** for throughput; all others use 115200.

## Wiring

### EMG sensor (MyoWare 2.0 → ESP32)

| MyoWare pin | ESP32 pin | ADC channel | Notes |
|---|---|---|---|
| VIN (+) | 3V3 | — | Power from ESP32 3.3 V rail |
| GND (−) | GND | — | Common ground |
| **ENV** | GPIO34 | ADC1_CH6 | Hardware envelope (rectified + 3.6 Hz LP). Gain adjustable via MyoWare pot. |
| **RAW** | GPIO35 | ADC1_CH7 | Amplified bipolar EMG, 20–500 Hz passband, centered at VCC/2. Fixed gain 200. |
| **RECT** | GPIO32 | ADC1_CH4 | Full-wave rectified (same bandpass as RAW, no smoothing). Fixed gain 200. |

GPIO34 and GPIO35 are input-only — ideal for analog. GPIO32 supports both. All three are ADC1 channels, which is the only ADC bank that works while WiFi/BLE is active.

### Motor driver (low-side N-MOSFET)

| Connection | Notes |
|---|---|
| ESP32 GPIO25 → 220 Ω → MOSFET **gate** | PWM (2 kHz, 8-bit) via LEDC. Gate resistor limits switching current + dampens slew. |
| MOSFET **source** → GND | Shared ground with ESP32 |
| MOSFET **drain** → motor (−) | Motor switched on the low side |
| 3V3 → motor (+) | Motor always sees +3.3 V |

MOSFET pinout (TO-92, flat side facing you): **1 = Source, 2 = Gate, 3 = Drain**.

### Circuit diagram

```
  ┌──────────────────┐                      ┌──────────────────┐
  │   MyoWare 2.0    │                      │      ESP32       │
  │                  │                      │  (WROOM-32UE)    │
  │  VIN  ───────────┼──────────────────────┤ 3V3              │
  │  GND  ───────────┼───────────┬──────────┤ GND              │
  │  ENV  ───────────┼───────────┼──────────┤ GPIO34 (ADC1_CH6)│
  │  RAW  ───────────┼───────────┼──────────┤ GPIO35 (ADC1_CH7)│
  │  RECT ───────────┼───────────┼──────────┤ GPIO32 (ADC1_CH4)│
  │                  │           │          │                  │
  └──┬──┬──┬─────────┘           │          │ GPIO25 ──┐       │
     │  │  │                     │          │          │       │
     │  │  └─ Reference (bony, e.g. elbow)  │          │       │
     │  └──── Mid-muscle electrode          │          │       │
     └─────── End-muscle electrode          │          │       │
                                            │        [220 Ω]   │
                                            │          ▼       │
                                            │      ┌───────┐   │
                                            │      │ 2N7000│   │
                                            │  G ──┤ gate  │   │
                                            │  S ──┤source │   │
                                            │  D ──┤ drain │   │
                                            │      └───┬───┘   │
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
```

## Build & flash

1. Open the sketch you want in Arduino IDE 2.x.
2. **Board:** *ESP32 Dev Module*. **Port:** `/dev/cu.usbserial-3110` (or whatever your board enumerates as).
3. **Upload Speed:** 921600.
4. Click Upload. Watch for `Leaving... Hard resetting via RTS pin...`.
5. Open Serial Monitor at the baud listed in the sketches table for the sketch you flashed.

## Phase 1 workflow — capture raw EMG data

For collecting datasets to tune the algorithm offline or validate hardware changes:

1. Flash `emg_button_capture/emg_button_capture.ino`.
2. Close Arduino IDE's Serial Monitor (it would steal the port from the logger).
3. In a terminal: `cd ../captures && ./capture.sh <subject_name>`.
4. The live dashboard opens. Short-press BOOT to start a session, tap BOOT at each rep onset, long-press to end.
5. Sessions land at `../captures/session_<subject>_<timestamp>.csv` and can be replayed through the Python analysis scripts in that folder.

Detailed protocol: [`../notes/data-capture-handshake.md`](../notes/data-capture-handshake.md).

## Phase 2 workflow — realtime cueing with the app

For the full Bioliminal Garment experience (live cueing during a set):

1. Flash `bicep_realtime/bicep_realtime.ino`.
2. Launch the Bioliminal mobile app. The app connects, subscribes to FF02, and writes commands to FF04.
3. Optionally, run `../captures/capture_ble.sh <subject>` in parallel to shadow-record the BLE stream for post-session analysis.

BLE protocol, algorithm spec, and app-side architecture: [`../notes/haptic-cueing-handshake.md`](../notes/haptic-cueing-handshake.md).

## Related docs

| File | What it covers |
|---|---|
| [`../notes/haptic-cueing-handshake.md`](../notes/haptic-cueing-handshake.md) | Phase 2 BLE protocol, algorithm (calibration window, rolling baseline, 15/25/50% thresholds, pose-compensation gate), app-side architecture. |
| [`../notes/data-capture-handshake.md`](../notes/data-capture-handshake.md) | Phase 1 foundation: MyoWare specs, FF02 packet format, raw-data-integrity principle. Carries forward into Phase 2 unchanged. |
| [`../notes/app-dev-handoff.md`](../notes/app-dev-handoff.md) | Onboarding prompt + v0 acceptance criteria for the app-side developer. |
| [`../notes/practitioner-notes-lawson.md`](../notes/practitioner-notes-lawson.md) | User-level × cue-channel matrix (beginner / intermediate / advanced). Drives the v1 UserProfile design. |
| [`../notes/emg-fatigue-explainer.md`](../notes/emg-fatigue-explainer.md) | Why peak amplitude and why 15%. |
| [`../notes/project-status-2026-04-16.md`](../notes/project-status-2026-04-16.md) | Current phase + overall roadmap. |

## Troubleshooting

- **Serial Monitor shows garbled characters:** baud mismatch. `emg_button_capture` uses 921600; everything else uses 115200.
- **"Port busy" when running the capture scripts:** Arduino IDE's Serial Monitor or another process is holding the port. Close all Serial Monitors before launching `capture.sh` / `capture_bench.sh`.
- **Motor fires but EMG reads 0 on all channels:** check the MyoWare VIN / GND solder joints. Confirm 3.3 V on VIN with a multimeter.
- **Motor doesn't fire on FF04 write:** test with the raw payload `10 00 B4 02 C8 00 96 00` in `BleDebugView`. If that works but app-side cues don't, the issue is app-side.
- **Gain pot won't turn:** leave it. RAW has a fixed gain of 200 that's independent of the pot; everything downstream uses software envelope from RAW and doesn't require the hardware ENV to be in a specific range.
