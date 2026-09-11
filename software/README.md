# Software

Single-file ESP32 firmware (`toots.ino`) written in C++ with the Arduino framework. It handles sensing, motor control, maze solving, and serves a live web dashboard — all from one sketch.

## Run Modes

The robot operates as a state machine (`RunMode`), driven from `solverStep()` in the main loop:

| Mode | Trigger | What it does |
|---|---|---|
| `MODE_IDLE` | Default / after any run finishes | Motors stopped, waiting for a command |
| `MODE_SOLVE` | `solve` command | Explores the maze, mapping walls with floodfill as it goes |
| `MODE_RETURN` | `return` command | Floods back to the start cell |
| `MODE_SPEEDRUN` | `speedrun` command (requires a completed map + robot at start) | Runs the shortest known path at full speed |

## Core Logic

- **`getWallCorrection()`** — compares left/right ToF readings against `WALL_THRESHOLD` and `SIDE_TARGET_MM` to compute a steering correction, keeping the robot centered (or a fixed distance from one wall)
- **`moveContinuousForward()`** — the main driving loop: reads encoders, computes error between wheels, runs it through a PID controller (`Kp`, `Ki`, `Kd`), adds the wall correction and a fixed motor bias (`BASE_CORRECTION`), then calls `setMotor()`
- **`floodFill(toStart, onlyVisited)`** — BFS distance-field solver; recomputes as new walls are discovered
- **`turnLeft90()` / `turnRight90()` / `turn180()`** — time-based turns using calibrated delays, followed by `postTurnCalibration()` to re-align using side wall distances

## Web Dashboard API

Served from the ESP32 itself — no external server needed.

| Endpoint | Method | Purpose |
|---|---|---|
| `/` | GET | Serves the dashboard HTML/CSS/JS |
| `/data` | GET | Returns live JSON: position, sensor readings, mode, all tunable parameters |
| `/cmd?c=...` | GET | Sends a command — see table below |
| `/set?k=...&v=...` | GET | Live-tunes a parameter without re-flashing |

**Commands (`/cmd?c=`):**

| Command | Effect |
|---|---|
| `solve` | Start maze exploration |
| `return` | Return to start cell |
| `speedrun` | Run the fastest known path |
| `s` | Emergency stop |
| `startfwd` / `stopfwd` | Manual continuous forward drive |
| `left90` / `right90` / `turn180` | Manual single turn |
| `celltof` | Move forward exactly one cell |
| `reset` | Zero the encoders |
| `resetpose` | Reset (x, y, direction) to origin |
| `resetmaze` | Clear the entire wall map |
| `debug` | Plain-text status dump over HTTP |

## Tunable Parameters

All adjustable live via the dashboard (`/set`), no re-upload needed:

| Parameter | Default | Role |
|---|---|---|
| `BASE_SPEED` | 120 | Cruising motor speed |
| `MIN_SPEED` | 58 | Floor speed after correction |
| `TURN_SPEED` | 120 | Speed during turns |
| `Kp` / `Ki` / `Kd` | 3.0 / 0.005 / 0.5 | PID gains for straight-line motor sync |
| `BASE_CORRECTION` | 3 | Fixed bias compensating for motor mismatch |
| `TURN_DELAY_LEFT` / `RIGHT` / `180` | 700 / 435 / 790 ms | Calibrated turn durations |
| `CELL_TICKS` | 250 | Encoder ticks per maze cell |
| `WALL_THRESHOLD` | 200 mm | Distance below which a wall is "detected" |
| `FRONT_STOP` | 100 mm | Distance to treat front as blocked |
| `SIDE_TARGET_MM` | 45 mm | Target distance from a single side wall |
| `LEFT_OFFSET` / `RIGHT_OFFSET` | -70 / -10 | Per-sensor calibration offsets |

## Known Limitations / Next Steps

- **Single monolithic file** — all logic lives in one `.ino`. Splitting sensing, motor control, and maze-solving into separate modules (`.h`/`.cpp` files) would make it easier to test and extend.
- **MPU-6050 not yet integrated** — the IMU is on the board but unused; turns currently rely on fixed time delays + post-turn wall calibration rather than gyro feedback. Adding gyro-based heading correction is a natural next step to reduce drift.
- **No automated tests** — all tuning and validation happens live via the dashboard.
