# Animatronic Pumpkin

Interactive 3D-printed pumpkin with LCD eyes, push detection, and a spring-loaded pop-up flame mechanism.

## Overview

The head sits on a central coil spring mounted to a weighted base. When pushed, an IMU detects the motion and progresses the eye graphics through 4 stages. On the 4th push, the top latch opens and a spring-loaded plunger ejects a lit 3D-printed flame.

### Hardware Specs
* **MCU:** ESP32
* **Displays:** 2x SPI LCDs (GC9A01 or ST7789) sharing an SPI bus with independent CS pins
* **Cocking Servo:** MG995 (180° positional) driving a half-toothed sector gear and linear rack
* **Latch Servo:** MG90S (180° micro servo) acting as a deadbolt for the top hatch
* **Sensor:** MPU6050 (or tilt switch) with software cooldown to ignore spring rebound
* **Power:** Battery pack and buck converters (5V logic, 6V servo rail) mounted in the bottom base

---

## Mechanism Cycle

1. **Idle / Charging (Pushes 1–3):** Hatch is closed and locked by the MG90S. The internal plunger spring is compressed against the hatch. MG995 sits at `180°` (unengaged).
2. **Trigger (Push 4):** MG90S unlocks the hatch. The plunger shoots the flame up, LEDs strobe, and the buzzer sounds. While the flame is at the top of its stroke, the rack clears the gear arc and the MG995 resets from `180°` to `0°`.
3. **Retract:** After 3 seconds, the MG995 sweeps from `0°` to `160°`, engaging the rack to pull the flame back down. It holds position at `160°` on the last gear tooth.
4. **Manual Reset:** Close the top hatch manually. When the hatch switch triggers, the MG90S locks the deadbolt, and the MG995 steps from `160°` to `180°` to release the gear teeth and transfer tension back to the hatch.

---

## Repo Structure

```text
├── cad/
│   ├── assembly.scad           # Main OpenSCAD file
│   └── exports_stl/            # Printable STL files
├── firmware/
│   ├── platformio.ini          # PlatformIO config
│   └── src/                    # ESP32 source code
├── docs/
│   ├── bom.md                  # Parts list
│   └── pinout.md               # Wiring reference
└── README.md
```

---

## Setup & Usage

### CAD (OpenSCAD)
1. Install the **OpenSCAD Development Snapshot** (not the 2021 stable release).
2. Go to **Edit -> Preferences -> Advanced** and enable the **Manifold** backend so rendering doesn't lag.
3. Open `cad/assembly.scad`.
4. Change `render_mode` at the top of the file to switch between `"assembly"` and the individual print plates (`"print_base"`, `"print_head_front"`, `"print_mechanism"`, etc.).
5. Press `F5` to preview, `F6` to render, and `F7` to export the STL.

### Firmware (PlatformIO)
1. Open the `firmware/` folder in VS Code with the PlatformIO extension installed.
2. Check pin definitions in `src/config.h`.
3. Build and flash:
   ```bash
   pio run -t upload
   ```