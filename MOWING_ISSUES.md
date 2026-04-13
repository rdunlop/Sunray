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

### GPS Serial Data Loss (SERIAL_BUFFER_SIZE)

The u-blox F9P GPS receiver communicates over Serial4 at 115200 baud. The Grand Central M4's hardware serial receive buffer is controlled by `SERIAL_BUFFER_SIZE` in the Arduino library file:

```
~/Library/Arduino15/packages/adafruit/hardware/samd/1.7.13/cores/arduino/RingBuffer.h
```

**This library file must be edited directly** — changing `config.h` alone does not resize the actual hardware buffer. The `#define SERIAL_BUFFER_SIZE 350` in `config.h` is only a constant the firmware code uses to report/verify the intended size; it has no effect on the Arduino serial driver unless the library file is also changed.

Robin's `RingBuffer.h` has already been set to 350 bytes. Robin has tried adjusting this value and observed no noticeable improvement. At 115200 baud, 350 bytes represents only ~30 ms of incoming data. If any operation in the main control loop (SD card write, BLE transmission, etc.) holds the CPU for longer than that without checking the GPS serial port, incoming GPS bytes are silently dropped. Dropped bytes produce checksum errors (`gps_chk_err` in `AT+T`, column 13) or corrupt NMEA sentences, which the firmware discards, causing GPS position gaps.

**Multiple simultaneous serial ports** are active on the Grand Central M4 in this configuration:
- `Serial` (USB) — console
- `Serial2` — WiFi/ESP8266 module
- `Serial3` — BLE module
- `Serial4` — GPS (u-blox F9P)

Each port has its own 350-byte ring buffer. SD card operations (SPI bus) can also hold the CPU for variable amounts of time. Whether these are actually causing GPS data loss is unknown and should be investigated (see "Things to Try" below).

The upstream recommendation is to use **1024 bytes** for `SERIAL_BUFFER_SIZE`. Robin reduced it to 350, which may have been intended to save RAM, but could be contributing to the problem if the control loop has any slow operations.

---

## Run Log

### 2026-04-13 — First run of the season

**Observed:** Mower was pointing west-NW but the app showed it believing it was pointing east. User pressed Stop and connected USB serial.

**Serial console input quirk:** When typing `AT+S`, the console echoed `CON:AT+SAT+S` — the firmware's console buffer had a leftover `AT+S` from a previous incomplete attempt, and the new `AT+S` was appended to it, producing `AT+SAT+S` which is an unrecognised command. This repeated until the buffer flushed. On the third attempt `AT+S` processed cleanly. Same pattern happened before `AT+T`. This is expected behaviour: the console accumulates characters until a newline and prints them with a `CON:` prefix. **Workaround:** wait for the previous echo to complete before sending the next command, or send a lone Enter to flush any stale partial command from the buffer.

#### AT+S decoded (at rest, after Stop):
```
S,27.54,4.93,-0.08,-0.19,2,0,0,0.87,9,5.42,-0.02,0.02,36,0.05,32,1011420,-0.51,-1,0,0xc4
```

| Field | Value | Interpretation |
|-------|-------|----------------|
| bat_v | 27.54 V | Battery fine |
| x, y | 4.93, -0.08 m | Position |
| **delta** | **-0.19 rad** | **Firmware thinks: ~east (−11°)**. Mower was actually facing west-NW ≈ +2.75 rad. **Difference ≈ 168° — nearly a perfect 180° flip.** |
| gps_sol | 2 | RTK fix — GPS currently good |
| op | 0 | IDLE (user had pressed Stop) |
| dgps_age_s | 0.87 s | RTK corrections very fresh |
| **sensor** | **9** | **`SENS_GPS_INVALID` — last error was GPS declared invalid** (now recovered) |
| gps_acc | 0.02 m | Excellent GPS accuracy |
| sv | 36 | 36 satellites in view |
| sv_dgps | 32 | 32 satellites contributing to RTK — strong RTK link |
| lateral_err | -0.51 m | Half a metre off the planned line when stopped |

**Confirmed from verbose serial output:** sol=2, age=0.34 during idle polling — GPS is fine now. The orientation error occurred and persisted during the mow.

#### AT+T decoded (covering the full session):
```
T,118,0,134,0,134,0,13.45,1.63,0,998,998,28,0,0.02,350,0,1,0,235939,1,0,0,0,0,0,0,0,0,0,0,0,0x68
```

| Field | Value | Interpretation |
|-------|-------|----------------|
| idle_s | 118 | ~2 min idle |
| mow_s | 134 | ~2 min 14 sec mowing total |
| mow_float_s | 0 | Zero seconds with degraded GPS during mow |
| mow_fix_s | 134 | **All 134 s of mowing at RTK fix** — GPS position quality was never the issue |
| mow_dist_m | 13.45 m | 13.5 m travelled |
| max_dgps_age_s | 1.63 s | Corrections never stale — RTK link was solid |
| **gps_chk_err** | **28** | **28 GPS NMEA checksum errors — serial line noise or EMI from motors corrupting GPS sentences** |
| dgps_chk_err | 0 | No RTK correction corruption |
| max_ctl_cycle_s | 0.02 | 20 ms — control loop fine, NOT causing buffer overflow |
| serial_buf_sz | 350 | Confirms buffer size |
| mow_invalid_recov | 1 | **GPS went invalid once during mow and triggered a recovery attempt** |
| gps_jumps | 0 | No GPS position jumps — position data was stable |
| gps_no_speed_cnt | 0 | GPS always reported motion speed — heading derivation had speed data |
| rot_timeout_cnt | 0 | No stuck rotation manoeuvres |
| heading_snaps | — | Not present in firmware at time of this run |
| heading_snap_max_deg | — | Not present in firmware at time of this run |

#### Analysis

1. **GPS position quality was excellent throughout.** RTK fix for all 134 s of mowing, corrections fresh, 32/36 satellites. The 180° heading error is not explained by GPS position being wrong.

2. **28 NMEA checksum errors, but max control cycle only 20 ms.** The serial buffer (350 bytes ≈ 30 ms at 115200 baud) should not have overflowed with 20 ms max cycles. The checksum errors are likely caused by **EMI from the three drive motors corrupting the GPS UART serial line**, not by buffer overflow. The firmware correctly rejects these sentences (that is what `gps_chk_err` counts). However, the NMEA checksum is a single XOR byte — a corrupted sentence has roughly a 1-in-256 chance of accidentally passing the checksum and being accepted as valid data.

3. **One `mow_invalid_recov`** means at some point the firmware determined the GPS was invalid and attempted recovery. This matches the `sensor=9 (SENS_GPS_INVALID)` in the AT+S snapshot. The GPS recovered (fix was restored) but the heading was already wrong and remained wrong.

4. **`gps_no_speed_cnt = 0`** means the GPS was always reporting motion. The heading derivation from GPS velocity had data to work with. This makes the 180° flip more puzzling — the GPS was giving a consistently wrong velocity direction, not just reporting zero speed.

5. **Hypothesis:** During the one `mow_invalid_recov` event, the firmware lost confidence in GPS state and re-initialised heading from a bad GPS fix or a corrupted-but-passing-checksum sentence that had reversed velocity. Without an IMU to sanity-check the new heading, the reversed heading was accepted and never corrected.

---

### 2026-04-13 — Second run (kidnap + heading flip)

**Observed:** Mower was triggered "kidnapped", then ended up confused. At rest, app and console showed delta=1.26 rad (~NNE, compass ~18°) while mower was actually pointing SSW (~200°). ~180° flip.

#### Raw serial output at Stop:
```
0:7:42 ... delta=1.26 ... sol=2 age=0.33 ...
S,27.03,5.53,-11.00,1.26,2,0,10,0.17,0,5.58,-11.00,0.02,36,0.08,32,1011420,-0.18,-1,0,0x15
T,101,0,245,0,224,0,18.48,24.40,0,998.00,998.00,39,0,0.02,350,21,2,0,235115,5,1,0,0,0,0,0,0,0,0,0,0,13,169.80,0xba
```

#### AT+S decoded:
| Field | Value | Notes |
|-------|-------|-------|
| delta | **1.26 rad** | Firmware heading ~NNE (compass 18°) — actually pointing SSW (~200°), ~180° off |
| gps_sol | 2 | RTK fix at rest |
| op | 0 | IDLE |
| dgps_age_s | 0.17 | Corrections fresh at time of reading |
| sensor | 0 | No current error (cleared on stop) |
| gps_acc | 0.02 m | GPS accurate at rest |
| sv / sv_dgps | 36 / 32 | Good satellite counts |

#### AT+T decoded:
| Field | Value | Notes |
|-------|-------|-------|
| mow_s | 245 | ~4 min 5 s total |
| mow_fix_s | 224 | 224 of 245 s at RTK fix |
| mow_invalid_s | **21** | **21 s mowing with invalid GPS — worse than run 1** |
| mow_invalid_recov | **2** | **2 GPS invalid recovery events** |
| **max_dgps_age_s** | **24.40 s** | **RTK corrections went stale for 24 s (was 1.63 s run 1) — NTRIP dropout** |
| gps_chk_err | 39 | 39 serial checksum errors in 245 s (similar rate to run 1) |
| gps_jumps | 1 | One position jump |
| reset_cause | 5 | |
| **heading_snaps** | **13** | **Snap fired 13 times — chronic, not a one-off** |
| **heading_snap_max_deg** | **169.80°** | **One snap was 170° — this is the 180° flip event** |
| heading_snap_speed_blocked | — | Not present in firmware at time of this run |

#### Analysis

1. **The heading snap is confirmed as the flip mechanism.** `heading_snaps=13` proves it fires repeatedly. `heading_snap_max_deg=169.80` captures the ~180° disaster. The snap at line 389 of `StateEstimator.cpp` is the root cause.

2. **NTRIP dropout is the trigger.** `max_dgps_age_s=24.40` s means RTK corrections were absent for 24 seconds at some point (was 1.63 s run 1). Without corrections the GPS degrades, heading data becomes unreliable, and the snap fires on noise. 21 s of invalid GPS and 2 recovery events follow from this.

3. **Serial EMI is ongoing.** 39 checksum errors — cabling has not yet been fixed. However, EMI alone is not the primary cause here: the NTRIP dropout is worse. Both need to be fixed independently.

4. **Speed gate now in firmware.** A speed gate (`stateGroundSpeed > 0.2 m/s`) has been added so that snaps are blocked at low speed (startup, recovery from stop). The `heading_snap_speed_blocked` counter will show how many snaps this suppresses in future runs.

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
| 33 | `heading_snaps` | Times the GPS heading snap (>45°) fired and was allowed through |
| 34 | `heading_snap_max_deg` | Largest single heading snap seen (degrees) |
| 35 | `heading_snap_speed_blocked` | Snaps suppressed by speed gate (stateGroundSpeed ≤ 0.2 m/s) |
| 36 | `0xHH` | CRC checksum |

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

**Status after 2026-04-13 run:**
- ✅ Ruled out: GPS position quality (all 134 s at RTK fix, fresh corrections, strong satellite counts)
- ✅ Ruled out: Control loop overflow causing serial buffer issues (max 20 ms, well under 30 ms threshold)
- ✅ Ruled out: GPS speed dropout (gps_no_speed_cnt = 0)
- ✅ Ruled out: Stale RTK corrections (max_dgps_age_s = 1.63 s)
- ⚠️ Confirmed: 28 GPS checksum errors — serial line noise / EMI from motors
- ⚠️ Confirmed: GPS went invalid once (mow_invalid_recov = 1) and heading was wrong afterwards
- ❓ Unknown: Whether the NMEA errors are causing rare corrupted-but-passing sentences that snap the heading

---

1. **Investigate GPS serial line EMI.** `gps_chk_err = 28` during only 134 s of mowing is high. This suggests the GPS UART wiring is picking up interference from the motor PWM signals. Check that the GPS serial cable is not routed near motor wires. Consider adding a ferrite bead to the GPS serial line. Check whether errors increase when the mow motor is actively spinning vs idle.

2. **Understand the `mow_invalid_recov` event.** This is the most likely trigger for the heading flip. The firmware declared GPS invalid and attempted recovery. When GPS was re-acquired, the heading was wrong and never corrected. It would be worth finding in `robot.cpp` what the firmware does with heading state during a GPS invalid → recovery cycle — does it reset heading from the first new GPS velocity, or does it preserve the last known heading?

3. **Check RTK link quality at the start of a mow.** Run `AT+S` right after mowing begins and look at column 14 (`sv`) vs column 16 (`sv_dgps`). On 2026-04-13 these were 36 and 32 — excellent. Worth confirming this remains true in future runs.

4. **Consider increasing `SERIAL_BUFFER_SIZE` to 1024.** Even though the 20 ms control loop is not overflowing the 350-byte buffer, 28 checksum errors in 2 minutes suggests the GPS line already has noise. A larger buffer (more headroom) can't hurt and might absorb any occasional timing spikes. Edit `~/Library/Arduino15/packages/adafruit/hardware/samd/1.7.13/cores/arduino/RingBuffer.h` and set `#define SERIAL_BUFFER_SIZE 1024`, then also update `config.h` to match. Note that changing only `config.h` has no effect.

5. **Implement the on-board diagnostic buffer** (see Firmware Modification Plan section). With a 120-second frozen buffer readable after a failure, the next run will show exactly what was happening second-by-second around the heading flip event — specifically whether GPS sol or delta changed suddenly at the moment of the mow_invalid_recov.

6. **Re-examine IMU possibility.** The gyroscope (yaw rate) part of an IMU would provide a sanity check: if delta changes by 180° in one control cycle without the gyro agreeing, the firmware could reject the bad reading. The magnetometer/compass is still suspect due to motor interference. Gyro-only heading integration with periodic GPS correction may be worth investigating.
