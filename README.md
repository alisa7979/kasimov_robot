# kasimov_robot

Firmware for an autonomous sumo robot, built for the KAsimov Cup competition (team 3조).

- **Board:** Arduino Uno
- **Motor driver:** L298N
- **Motors:** 2x GM20-2025-12100 (12V, 2.5W spur gear DC)
- **Sensing:** 2x front-facing HC-SR04 ultrasonic sensors, 4x corner-mounted line tracers
- **Control:** Priority-cascade finite state machine — edge avoidance is the highest-priority state, overriding attack/search behavior whenever a line sensor trips

> Side IR sensors were part of an earlier design iteration and have since been removed — they caused false-attack events during testing. They no longer exist in the current hardware or firmware.

## Repo structure

```
kasimov_robot/
├── main/
│   ├── main_ver4.ino     ← current firmware, flash this
│   ├── main_ver3.ino      (previous iteration, kept for reference)
│   └── main_ver2.ino      (previous iteration, kept for reference)
├── test/                  ← isolated per-subsystem test sketches
│   ├── UC_test/            ultrasonic sensor read/filter tests
│   ├── line_test/           line tracer threshold + edge detection tests
│   ├── motor_test/          L298N driver / PWM sanity checks
│   ├── trace_test/          combined line-tracer behavior tests
│   ├── turn_test/            in-place turning / tank-drive tests
│   └── main_ver1/            earliest full integration test
└── archive/                retired code (old sensor approaches, deprecated modules)
```

## Where to start

- **Flashing the robot:** open `main/main_ver4.ino`. This is the only file that should be uploaded to the Uno.
- **Understanding or debugging a subsystem:** check the matching folder under `test/` first. Each one isolates a single piece of hardware (ultrasonic, line tracers, motors, turning) so you can verify it in isolation before touching `main_ver4.ino`. These are the most reliable reference for how each sensor/actuator is actually wired and driven, since `main_ver4.ino` only calls into the behavior — it doesn't re-explain the low-level details.
- **Old approaches (e.g. IR-based targeting, earlier ultrasonic filtering):** see `archive/`. Not used by current firmware.

## FSM priority order (`main_ver4.ino`)

1. **Edge avoidance** — any of the 4 corner line tracers trips → immediate escape maneuver, overrides everything else.
2. **Attack / opponent tracking** — front ultrasonic sensors detect an opponent within range.
3. **Search** — no opponent detected, sweep/patrol behavior.

## Setup

1. Open `main/main_ver4.ino` in the Arduino IDE.
2. Verify pin mapping matches your wiring (see comments at top of file / corresponding `test/` sketch for each sensor).
3. Upload to Arduino Nano.
4. If behavior looks off, isolate the issue using the relevant `test/` sketch before debugging in `main_ver4.ino`.
