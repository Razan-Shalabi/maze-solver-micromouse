# 🐭 Toots — Autonomous Micromouse

An autonomous maze-solving robot built from scratch — custom 3D-printed chassis, ESP32 firmware in C++, and a floodfill algorithm that finds its way through an 8×8 maze without any human input.

**Team:** Razan Shalabi · Shatha Abualrub · Lara Daifallah · Ghada Swalha
**Instructor:** Wasel Ghanem
**University:** Birzeit University

---

## What This Project Does

Toots navigates an 8×8 cell maze on its own. It:

1. **Senses** walls in real time using three VL53L0X ToF sensors (front, left, right)
2. **Tracks movement** using magnetic wheel encoders on both N20 motors
3. **Plans** the shortest path using the Floodfill (BFS) algorithm
4. **Drives** with PID-corrected motor control, staying centered between walls as it goes
5. **Hosts a live web dashboard** — connect over WiFi and watch it think in real time, or tune parameters on the fly

Everything — wiring, chassis design, and firmware — was designed and built by the team.

## Documentation

| Resource | Link |
|---|---|
| 🔧 Hardware & Components | [`hardware/README.md`](hardware/README.md) |
| 🖨️ 3D Models (STL + Fusion 360) | [`hardware/3d-models/`](hardware/3d-models/README.md) |
| 💻 Firmware & Architecture | [`software/README.md`](software/README.md) |
| 📋 Full Build Log | [Trello Board](https://trello.com/invite/b/69e2683745c4a1b255d148a3/ATTI001aa2c3a295a99a65e10da04925d9e88B10C430/interface-project) |

## Hardware Stack

| Component | Part | Role |
|---|---|---|
| MCU | ESP32 Dev Kit | Runs firmware + web dashboard |
| Motor Driver | DRV8833 | Drives both N20 motors |
| Motors | N20 DC w/ Encoder, 6V 530 RPM | Differential drive + speed feedback |
| ToF Sensors ×3 | VL53L0X | Wall detection: front, left, right |
| Chassis | Custom 3D-printed | Compact, maze-optimized |

Full parts list with photos and wiring: [`hardware/README.md`](hardware/README.md)

## How It Works

- **Wall sensing** — three ToF sensors feed `getWallCorrection()`, which steers the robot to stay centered between walls (or a fixed distance from a single wall) while driving forward
- **Movement tracking** — interrupt-driven encoder ticks on both wheels feed a PID loop that keeps the two motors in sync
- **Path planning** — a floodfill (BFS) solver computes the shortest path to the maze center as it explores, and re-solves as new walls are discovered
- **WiFi setup** — no hardcoded credentials. On first boot, the ESP32 opens a `TOOTs-Setup` access point; connecting to it lets you pick your WiFi network and enter the password once, saved to flash for every future boot
- **Live dashboard** — reachable at `http://toots.local` (or by IP), showing live sensor readings, position, and mode, with sliders to tune PID/turn/wall parameters without re-flashing

Full breakdown of run modes, the dashboard API, and every tunable parameter: [`software/README.md`](software/README.md)

## Quick Start

**Prerequisites:**
- Arduino IDE with the ESP32 board package installed
- Libraries: `WiFiManager` (tzapu), `VL53L0X`, `ESPmDNS` (built into the ESP32 core)

**Steps:**
1. Open `software/toots.ino` in the Arduino IDE
2. Select your ESP32 board and port
3. Upload
4. On first boot, connect to the `TOOTs-Setup` WiFi network it creates, and enter your home WiFi credentials
5. Visit `http://toots.local` (or the IP shown in Serial output) to see the live dashboard

## Repository Structure

```
maze-solver-micromouse/
├── README.md
├── software/
│   ├── README.md               ← architecture, modes, API, tunable parameters
│   └── toots.ino                ← firmware
└── hardware/
    ├── README.md                ← full parts table + pin map
    ├── components/               ← per-component docs (wiring, code usage, troubleshooting)
    └── 3d-models/                ← chassis STL + Fusion 360 link
```

## The Journey

It started with a pile of components arriving — motors, sensors, the ESP32, all still in their static bags. Once everything was in hand, the team sat down together to figure out how it would all fit together.

The chassis was modeled in Fusion 360 before ever touching a printer — laid out to fit within the maze cell constraints while staying light enough not to load down the N20 motors. It was then printed for free at Blue Dome, keeping the build's cost down.

Before the physical robot ever moved through a real maze, the floodfill (BFS) solving logic was tested in simulation, mapping out the shortest path across the 8×8 grid to confirm the algorithm worked before trusting it to real hardware.

With the chassis built, components wired, and the algorithm proven, it all came together into Toots — sensing its way through a maze with three ToF sensors, tracking its own movement, and finding the center without any human input.

Want the full day-by-day story, including early struggles and every decision along the way? Check out the [Trello board](https://trello.com/invite/b/69e2683745c4a1b255d148a3/ATTI001aa2c3a295a99a65e10da04925d9e88B10C430/interface-project).

## Security Note

This repository contains no credentials, API keys, or WiFi passwords. WiFi setup happens entirely on-device via WiFiManager's captive portal — nothing is hardcoded in the firmware.

## License

This project is submitted as academic coursework. You're welcome to study, fork, and learn from it — just note we're students and still learning, so double-check everything.
