# Mowing Issues — Robin's Setup

This document records known problems with Robin's mower, the suspected causes, and a data-gathering plan for diagnosing failures. It is intended to be read at the start of future conversations about mowing issues.

---

## Setup

- **Base station:** RTK GPS receiver mounted on the house, broadcasting corrections
- **Rover:** RTK GPS receiver on the mower, receiving corrections from base station (RTCM over BLE or radio link)
- **Board:** Adafruit Grand Central M4 (SAMD51) running Sunray firmware on the `robin_modifications` branch
- **IMU:** **None installed.** Several attempts to get useful values from a desktop IMU have failed. The IMU is not in the mower.
- **Lawn:** Has a pronounced tilt across the whole surface (not flat)
- **Cutter height adjustment:** Not installed on this machine.
- **Connectivity:**
  - **BLE (iPhone app):** The Sunray iOS app provides a limited UI for starting mowing, uploading maps, manual driving, and viewing mower orientation/status. It does **not** expose a console or raw AT command interface.
  - **USB Serial:** A USB cable can be connected to the Grand Central M4's USB port to open a serial console. From this terminal, AT commands (`AT+V`, `AT+S`, `AT+T`, etc.) can be typed manually and responses read. This is available but not practical for continuous monitoring during a mow (would require a laptop secured to the mower with a cable).

---

## The Problem

**Symptom:** The lawn does not get mowed. The mower sits in the lawn for an extended period (sometimes eventually recovering, sometimes not).

**Observed failure mode (via BLE app map view):** The mower believes it is moving in one direction (e.g., north) while it is visibly moving in a different direction (e.g., south-east). This is not a motor direction error — the wheels are turning the correct way relative to the robot's body — but the robot does not know its own orientation correctly. Once in this confused state, it does not reliably self-correct.

---

## Why This Happens (Suspected Root Causes)

### No IMU = No Independent Heading Reference

Without an IMU, the firmware must derive the robot's heading entirely from the **GPS velocity vector** (the direction the GPS position is changing over time). This works when:
- The mower is moving at a useful speed
- GPS gives accurate, consistent position updates
- RTK corrections are fresh (low `dgps_age`)

It fails when GPS signal is multipath-reflected, when the mower is barely moving, or when RTK quality degrades. A momentarily bad heading corrupts the robot's internal coordinate frame, and without an IMU to provide an independent check, the error can compound.

### Sloped Lawn Complicates IMU Use

Even if an IMU were installed, the lawn's pronounced tilt means the IMU would always read a non-zero pitch/roll. The firmware likely assumes a relatively flat surface for its sensor fusion. Additionally, three large motors on the mower would generate significant magnetic interference, making the IMU's compass (magnetometer) unreliable for heading — which is the one thing most needed.

### Kidnap Detection Without IMU

The firmware has kidnap detection (`SENS_KIDNAPPED`): if the robot's actual GPS position diverges significantly from its predicted track, it flags a "kidnap" and tries GPS recovery. Without IMU heading, the predicted track is itself uncertain, so the kidnap detection may either not trigger when it should, or trigger and fail to recover correctly.

---

## AT Commands (via USB Serial)

Connect a USB cable to the Grand Central M4 and open a serial terminal at 115200 baud. Commands are typed and responses read manually. No CRC is required when using the direct serial console (`CON` mode — the firmware does not enforce CRC on the physical console port).

### Real-time telemetry — `AT+S`

The response is a comma-separated list with **no column headers**. Fields in order:

| # | Field | Meaning |
|---|-------|---------|
| 1 | `S` | Response prefix |
| 2 | `bat_v` | Battery voltage (V) |
| 3 | `x` | Robot X position (m) |
| 4 | `y` | Robot Y position (m) |
| 5 | `delta` | Heading (radians): 0=east, 1.57=north, ±3.14=west, -1.57=south |
| 6 | `gps_sol` | GPS solution: 0=invalid, 1=float, 2=RTK fix |
| 7 | `op` | Operation: 0=IDLE, 1=MOW, 2=CHARGE, 3=ERROR, 4=DOCK |
| 8 | `mow_idx` | Current mowing waypoint index |
| 9 | `dgps_age_s` | Age of RTK correction data (s) — should stay < 5 |
| 10 | `sensor` | Current error/sensor code (see table below) |
| 11 | `tgt_x` | Target waypoint X (m) |
| 12 | `tgt_y` | Target waypoint Y (m) |
| 13 | `gps_acc` | Reported GPS accuracy (m) — should be < 0.05 with RTK fix |
| 14 | `sv` | Total satellites in view |
| 15 | `amps_or_chg` | Motor current (A), or negative charging current when charging |
| 16 | `sv_dgps` | Satellites contributing to RTK corrections — should be close to `sv` |
| 17 | `map_crc` | Map checksum |
| 18 | `lateral_err` | How far off the planned mow line (m) |
| 19 | `tt_day` | Timetable: next auto-stop/start day |
| 20 | `tt_hour` | Timetable: next auto-stop/start hour |
| 21 | `0xHH` | CRC checksum |

### Sensor/error codes (`sensor` field, column 10)
| Value | Name | Meaning |
|-------|------|---------|
| 0 | `SENS_NONE` | No error |
| 3 | `SENS_GPS_FIX_TIMEOUT` | GPS RTK fix lost for too long |
| 6 | `SENS_KIDNAPPED` | Robot position diverged too far from planned track |
| 11 | `SENS_MAP_NO_ROUTE` | Cannot find a route to the next planned waypoint |
| 25 | `SENS_GPS_INVALID` | GPS not working / no valid data |

Other codes exist (see `sunray/types.h`) but these are the most likely during orientation failures.

### Statistics snapshot — `AT+T`

The response is also unlabeled. Fields in order:

| # | Field | Meaning |
|---|-------|---------|
| 1 | `T` | Response prefix |
| 2 | `idle_s` | Seconds spent idle |
| 3 | `charge_s` | Seconds spent charging |
| 4 | `mow_s` | Seconds spent mowing |
| 5 | `mow_float_s` | Seconds mowing with GPS float (degraded) |
| 6 | `mow_fix_s` | Seconds mowing with GPS RTK fix (good) |
| 7 | `float_to_fix` | Count of float→fix transitions |
| 8 | `mow_dist_m` | Total distance mowed (m) |
| 9 | `max_dgps_age_s` | Maximum RTK correction age seen (s) |
| 10 | `imu_recov` | IMU recovery count (N/A without IMU) |
| 11 | `tmin_C` | Minimum temperature (°C) |
| 12 | `tmax_C` | Maximum temperature (°C) |
| 13 | `gps_chk_err` | GPS NMEA checksum errors (serial corruption) |
| 14 | `dgps_chk_err` | RTK correction checksum errors |
| 15 | `max_ctl_cycle_s` | Longest control loop cycle (s) — should be < 0.03 |
| 16 | `serial_buf_sz` | Serial buffer size in use |
| 17 | `mow_invalid_s` | Seconds mowing with invalid GPS |
| 18 | `mow_invalid_recov` | Recovery attempts from invalid GPS |
| 19 | `mow_obstacles` | Obstacle count during mowing |
| 20 | `free_mem` | Free memory (bytes) |
| 21 | `reset_cause` | Last reset cause code |
| 22 | `gps_jumps` | GPS position jumps detected (multipath) |
| 23 | `sonar_cnt` | Sonar trigger count |
| 24 | `bumper_cnt` | Bumper trigger count |
| 25 | `gps_motion_to_cnt` | Times GPS motion timeout triggered |
| 26 | `mow_motor_recovery_s` | Mow motor recovery time (s) |
| 27 | `lift_cnt` | Lift sensor trigger count |
| 28 | `gps_no_speed_cnt` | Times GPS reported no speed (stalled heading) |
| 29 | `tof_cnt` | Time-of-flight sensor trigger count |
| 30 | `diff_imu_wheel_yaw_cnt` | IMU vs wheel yaw disagreement count (N/A without IMU) |
| 31 | `imu_no_rot_speed_cnt` | IMU rotation not detected (N/A without IMU) |
| 32 | `rot_timeout_cnt` | Rotation manoeuvre timeout count |
| 33 | `0xHH` | CRC checksum |

### Other useful commands
- **`AT+L`** — Clear all statistics. Run before a mowing session.
- **`AT+Y2`** — Reboot the u-blox F9P GPS receiver if it appears stuck.
- **`AT+C,-1,0,-1,-1,-1,-1,-1,-1,-1,-1,-1`** — Force IDLE (stop the mower safely).

---

## Data Gathering Plan — Current Approach (USB Serial, Manual)

### Before starting
1. Connect USB cable and open a serial terminal (115200 baud).
2. Type `AT+L` to clear statistics so the post-run `AT+T` reflects only this session.
3. Note the start time.
4. Disconnect and let the mower run (don't need cable during the mow).

### When a failure occurs or after the mow
1. Re-connect USB cable.
2. Type `AT+S` and copy the full raw response line. Use the field table above to decode it.
3. Type `AT+T` and copy the full raw response line.
4. Note what you observed (mower sitting still, driving wrong direction, spinning, etc.).
5. Note the `sensor` field value (column 10 in `AT+S`) and the `delta` value (column 5).
6. Compare `delta` to the mower's actual facing direction (use phone compass as reference): `0` = facing east, `1.57` = north, `-1.57` = south, `±3.14` = west.

---

## Firmware Modification Plan — On-Board Diagnostic Buffer

**Goal:** Without a continuously-connected laptop, the mower should record its own telemetry and make the recent history available on demand. This avoids needing to run anything externally during a mow.

### Concept

A circular buffer inside the firmware stores a snapshot of key telemetry fields once per second. After a triggering condition is detected (see below), the buffer stops updating and preserves the last N seconds of history. The buffer can then be read via a serial command after the mow.

### Triggering condition (Phase 1)

**Freeze the buffer when the mow blade motor turns off and stays off for more than 30 seconds.**

Rationale: if the blade stops mid-mow, something went wrong. The 30-second window filters out intentional pauses (e.g., obstacle avoidance). At that moment, the last 120 seconds of telemetry is the most relevant data.

### Buffer contents (per entry)
- Timestamp (seconds since mow start, or `millis()` / 1000)
- `x`, `y` — robot position (m)
- `delta` — heading (radians)
- `gps_sol` — GPS solution quality (0/1/2)
- `dgps_age_s` — RTK correction age (s)
- `sensor` — active error code
- `lateral_err` — lateral tracking error (m)
- `sv_dgps` — satellites with RTK corrections
- `op` — current operation

### Parameters
- **Interval:** 1 second per entry
- **Depth:** 120 entries (last 2 minutes before freeze)
- **Total storage:** ~120 × ~40 bytes = ~5 KB (well within the Grand Central M4's 256 KB RAM)

### Read command

New AT command: **`AT+DB`** — "Diagnostic Buffer dump"

Response: one line per buffer entry (oldest to newest), CSV format with a header row, followed by `OK`.

### Where to implement in the code

- New file: `sunray/DiagBuffer.h` / `DiagBuffer.cpp` — the buffer struct, write, freeze, and dump functions
- Hook into `sunray/robot.cpp` `Robot::run()`: call `diagBuffer.record(...)` each second
- Hook into mow motor state: when `motor.motorMowEnabled` transitions from true to false, start a 30-second countdown; freeze on expiry
- Add `cmdDiagBuffer()` to `sunray/comm.cpp` alongside the other `AT+` handlers

### Phase 2 (future)

Once Phase 1 is working and the output is useful, consider:
- **Multiple freeze triggers:** also freeze on `SENS_KIDNAPPED`, `SENS_GPS_FIX_TIMEOUT`, `SENS_MAP_NO_ROUTE`
- **Auto-write to SD card** on freeze (rather than waiting for a manual read command)
- **Configurable depth:** use the `AT+C` `<height>` field (unused on this machine) to set buffer depth at runtime (e.g., `height=60` → 60 seconds, `height=120` → 120 seconds)

---

## The Cutter Height Hack

Since the machine has no cutter height motor, the `<height>` field in `AT+C` commands (`AT+C,-1,-1,-1,-1,-1,-1,-1,-1,-1,<height>,-1`) does nothing mechanically. This value is passed through to `motor.cutHeight` but not acted upon. It can be repurposed as a runtime flag or numeric parameter to control custom firmware behavior (e.g., diagnostic buffer depth in Phase 2 above). This would require reading `motor.cutHeight` in the new code.

---

## Things to Try / Investigate

1. **Check RTK link quality at the start of a mow:** Run `AT+S` right after mowing begins and look at column 14 (`sv`) vs column 16 (`sv_dgps`). The `sv_dgps` count should be close to `sv`. If it's much lower, the base station link is weak or the base station itself doesn't have a solid fix.

2. **Check `dgps_age_s` (column 9 in `AT+S`) during problem moments:** If it spikes above 10 s, RTK corrections are not arriving. The rover degrades from RTK fix → float → invalid, which causes heading loss.

3. **Look at `gps_jumps` (column 22 in `AT+T`) and `gps_chk_err` (column 13 in `AT+T`):** High values indicate GPS signal quality problems (multipath, serial corruption) rather than a software logic issue.

4. **Look at `gps_no_speed_cnt` (column 28 in `AT+T`):** This counts how often the GPS reported zero or near-zero speed. When speed is too low, the GPS velocity vector becomes meaningless for heading — this is directly the heading-loss failure mode.

5. **Look at `rot_timeout_cnt` (column 32 in `AT+T`):** If non-zero, the mower was unable to complete turns, which could mean it's physically stuck or the heading error is making it overshoot turns repeatedly.

6. **Re-examine IMU possibility:** Even a noisy or tilted IMU that provides usable *relative* heading changes (gyroscope yaw rate) might help more than nothing. Gyro drift over a 1-hour mow may be tolerable. The magnetometer/compass part of the IMU is the questionable piece given motor magnetic interference. A gyro-only heading integration (ignoring the magnetometer) might be worth experimenting with.
