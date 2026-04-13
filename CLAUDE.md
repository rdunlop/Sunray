# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What This Is

Sunray is open-source RTK-GPS-based autonomous lawn mower firmware. The same core codebase compiles for three targets: Arduino MCUs (Due or Adafruit Grand Central M4), Linux single-board computers (Alfred/owlRobotics platform), and a software simulator.

## Build Systems

### Arduino (MCU target — Ardumower hardware)
Built and flashed using **Arduino IDE** only — there is no CLI build system for the MCU target. The entry point is `sunray/sunray.ino`. All `.cpp` files in `sunray/` are compiled alongside it.

### Linux (Alfred / owlPlatform / Simulator)
```bash
cd linux
cp config_alfred.h config.h    # or config_owlmower.h / config_sim.h
mkdir -p build && cd build
cmake ..
make
sudo ./sunray                   # root needed for serial/GPIO/port 80
```

The Linux build excludes files matching `agcm4|due|esp` (Arduino-only code). The firmware source is pulled from `../sunray/` via `FIRMWARE_PATH` in `CMakeLists.txt`.

### ESP32 BLE bridge
Built with **Arduino IDE** using `esp32_ble/esp32_ble.ino` as the entry point. This is a standalone companion firmware, not part of the main build.

### ROS (experimental, Docker-based)
```bash
cd ros && ./service.sh    # interactive menu for Docker setup and build
```

## Configuration

`sunray/config.h` is the single file that controls everything — platform selection, pin assignments, feature flags, and hardware specs. It is created by copying `sunray/config_example.h`. The file is tracked in this repo (`.gitignore` has been modified to allow it).

`esp32_ble/config.h` is the equivalent for the ESP32 firmware.

**Platform is selected by which `#define` is active at the top of `config.h`:**
- `DRV_ARDUMOWER` — direct MCU GPIO/PWM (default, Ardumower hardware)
- `DRV_SERIAL_ROBOT` — Alfred (STM32 MCU via serial UART)
- `DRV_CAN_ROBOT` — owlRobotics platform (CAN-bus)
- `DRV_SIM_ROBOT` — software simulator

**This repo's `config.h` is set up for Robin's specific robot:** Adafruit Grand Central M4, SAMD51 board, 42mm green-connector motors (696/2 ticks/rev), WHEEL_BASE_CM=37, WHEEL_DIAMETER=350, sonar enabled, SD card + SD resume enabled, overload and RPM fault detection disabled.

## Architecture

### Driver Abstraction (sunray/src/driver/)
Four concrete driver classes implement the same abstract interfaces (`RobotDriver`, `MotorDriver`, `BatteryDriver`, etc.), selected at compile time. `AmRobotDriver` does direct MCU GPIO. `SerialRobotDriver` and `CanRobotDriver` communicate with separate hardware controllers over serial/CAN. `SimRobotDriver` simulates everything in software.

### Main Control Loop (sunray/robot.cpp)
`Robot::run()` executes at ~50 Hz and orchestrates everything in sequence: NTRIP client, hardware driver polling, GPS parsing, state estimation, path planning, operation state machine, obstacle detection, and all communication channels (BLE, HTTP, MQTT, AT commands).

### State Estimator (sunray/StateEstimator.cpp)
Sensor fusion combining RTK-GPS, IMU (MPU6050/BNO055/ICM-20948), and motor encoder odometry into robot pose (`stateX`, `stateY`, `stateDelta`). Supports multiple localization modes: GPS, IMU+odometry dead reckoning, April Tag vision, LiDAR reflector, and guidance sheet.

### Operation State Machine (sunray/src/op/)
Each `.cpp` file is one operation: `MowOp`, `DockOp`, `ChargeOp`, `IdleOp`, `ErrorOp`, escape/recovery ops, GPS wait ops. Operations transition by returning the next `Op*` from their `run()` method.

### Map & Path Planning (sunray/map.cpp)
Handles lawn polygon storage, mowing pattern generation (parallel lines / spiral), obstacle map (grid-based), exclusion zones, and A*-style routing. This is the largest single file (~1600+ lines).

### Communication (sunray/comm.cpp)
Implements the AT command protocol used by the phone app over BLE or WiFi. All app↔robot communication goes through `Comm::processCmd()`.

## Directory Map

| Directory | Purpose |
|-----------|---------|
| `sunray/` | Core firmware — compiles for both Arduino and Linux |
| `sunray/src/driver/` | Platform driver implementations (4 variants) |
| `sunray/src/op/` | Operation state machine states |
| `sunray/src/ublox/` | u-blox F9P RTK GPS driver |
| `sunray/src/ntrip/` | NTRIP client for RTK corrections |
| `sunray/src/mpu/`, `bno/`, `icm/` | IMU drivers |
| `linux/` | Linux-specific build system, HAL, and platform configs |
| `linux/src/` | Linux implementations of Arduino hardware abstractions |
| `esp32_ble/` | Standalone ESP32 BLE/WiFi UART bridge firmware |
| `ros/` | Experimental ROS/Docker integration with LiDAR SLAM |
| `doc/` | AT protocol documentation, hardware notes |

## Key Conventions

- **Arduino IDE compilation units**: `sunray.ino` + all `.cpp`/`.h` in `sunray/` are compiled as one unit by the IDE. The `.ino` file is the entry point; everything else is included implicitly.
- **Linux vs Arduino ifdefs**: Platform-specific code is gated on `__linux__`, `__SAM3X8E__` (Arduino Due), `__SAMD51__` (Grand Central M4). The Linux build uses `linux/src/` to stub out Arduino library calls.
- **No unit test framework**: Testing is done via built-in AT commands (`AT+T` triggers sensor tests) and the simulator (`DRV_SIM_ROBOT` with keyboard interaction).
- **Serial console**: On Grand Central M4, `CONSOLE` is `Serial` (USB). On Arduino Due it defaults to `SerialUSB` (native port).
