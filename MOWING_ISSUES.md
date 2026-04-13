# Mowing Issues — Robin's Setup

This document records known problems with Robin's mower, the suspected causes, and a data-gathering plan for diagnosing failures. It is intended to be read at the start of future conversations about mowing issues.

---

## Setup

- **Base station:** RTK GPS receiver mounted on the house, broadcasting corrections
- **Rover:** RTK GPS receiver on the mower, receiving corrections from base station (RTCM over BLE or radio link)
- **Board:** Adafruit Grand Central M4 (SAMD51) running Sunray firmware on the `robin_modifications` branch
- **IMU:** **None installed.** Several attempts to get useful values from a desktop IMU have failed. The IMU is not in the mower.
- **Lawn:** Has a pronounced tilt across the whole surface (not flat)
- **Connectivity:** iPhone BLE only via the Sunray app. No serial/USB connection to the mower.
- **Cutter height adjustment:** Not installed on this machine.

---

## The Problem

**Symptom:** The lawn does not get mowed. The mower sits in the lawn for an extended period (sometimes eventually recovering, sometimes not).

**Observed failure mode (via BLE app):** The mower believes it is moving in one direction (e.g., north) while it is visibly moving in a different direction (e.g., south-east). This is not a motor direction error — the wheels are turning the correct way relative to the robot's body — but the robot does not know its own orientation correctly. Once in this confused state, it does not reliably self-correct.

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

## AT Commands Available via BLE

The Sunray app exposes a console. These commands are relevant for diagnosis:

### Real-time telemetry — `AT+S`
Poll this repeatedly to watch live state. Key fields in the response:
```
S, bat_v, x, y, delta, gps_sol, op, mow_idx, dgps_age_s, sensor,
   tgt_x, tgt_y, gps_acc, sv, amps, sv_dgps, map_crc, lateral_err, ...
```
- **`delta`** — heading in radians. Watch for sudden jumps (e.g. ±π flip). `0` = east, `π/2` ≈ 1.57 = north, `π` = west, `-π/2` ≈ -1.57 = south.
- **`gps_sol`** — GPS solution quality: `0`=invalid, `1`=float, `2`=fix. Should be 2 (RTK fix) when mowing.
- **`dgps_age_s`** — Age of RTK correction data in seconds. If this climbs above ~5–10 s, RTK quality is degrading.
- **`sensor`** — Current error/sensor code (see table below).
- **`gps_acc`** — Reported GPS accuracy in meters. Should be <0.05 m with RTK fix.
- **`sv` / `sv_dgps`** — Total satellites / satellites with RTK corrections. Low `sv_dgps` means the base station link is weak.
- **`op`** — Current operation: `0=IDLE`, `1=MOW`, `2=CHARGE`, `3=ERROR`, `4=DOCK`.
- **`lateral_err`** — How far off the planned line the mower currently is (meters).
- **`x`, `y`** vs **`tgt_x`, `tgt_y`** — Robot's believed position vs current waypoint target.

### Sensor/error codes (`sensor` field in `AT+S`)
| Code | Meaning |
|------|---------|
| 0 | No error |
| 3 | `SENS_GPS_FIX_TIMEOUT` — GPS fix lost too long |
| 6 | `SENS_KIDNAPPED` — robot diverged from planned path |
| 11 | `SENS_MAP_NO_ROUTE` — cannot find path to next waypoint |
| 25 | `SENS_GPS_INVALID` — GPS not working |

### Statistics snapshot — `AT+T`
Run this after a failed mowing session. Key fields:
- **`gps_chk_err`** — GPS NMEA checksum errors (data corruption on GPS serial link)
- **`dgps_chk_err`** — RTK correction checksum errors
- **`gps_jumps`** — Times the GPS position jumped unexpectedly (multipath / signal bounce)
- **`gps_motion_to_cnt`** — Times the mower wasn't moving when it should have been
- **`mow_invalid_s`** — Seconds spent mowing with invalid/degraded GPS
- **`mow_invalid_recov`** — Recovery attempts from invalid GPS state
- **`rot_timeout_cnt`** — Times a rotation manoeuvre timed out (couldn't complete a turn)
- **`mow_obstacles`** — Obstacle count during mowing

### Clear statistics — `AT+L`
Run this **before** a mowing session so the statistics after reflect only that session.

### Force reboot GPS — `AT+Y2`
If the GPS appears to be stuck, this reboots the u-blox F9P receiver.

### Force idle — `AT+C,-1,0,-1,-1,-1,-1,-1,-1,-1,-1,-1`
Send `op=0` (IDLE) to safely stop the mower if it's stuck.

---

## Data Gathering Plan for Tomorrow's Run

### Before starting
1. Open Sunray app console and run `AT+L` to clear all statistics.
2. Note the time you start.

### While mowing (monitor periodically)
Watch `AT+S` every 30–60 seconds and note:
- Is `gps_sol` staying at `2` (RTK fix)?
- Is `dgps_age_s` staying low (< 5 s)?
- Is `delta` (heading) stable, or jumping?
- Is `lateral_err` growing?

### When a failure occurs (mower stops / gets confused)
Record as many of the following as possible immediately:
1. **`AT+S` full response** — copy the raw output
2. Time since mowing started
3. What the mower was visibly doing (sitting still, spinning, driving wrong direction)
4. What the `sensor` field shows
5. What `delta` shows vs what direction the mower was actually pointing (use compass on phone to get a rough actual heading)
6. What `gps_sol` and `dgps_age_s` show
7. Whether the BLE app map shows the mower dot in a plausible position

### After the session (or after a failure)
1. Run `AT+T` and copy the full response
2. If the SD card logs are accessible (via the Sunray app log viewer), download them

---

## The Cutter Height Hack

Since the machine has no cutter height motor, the `<height>` field in `AT+C` commands (`AT+C,-1,-1,-1,-1,-1,-1,-1,-1,-1,<height>,-1`) does nothing mechanically. This value is passed through but not acted upon. If future debugging requires toggling a software flag without a serial connection, the height field could potentially be used as a channel to send values to custom firmware code. This would require a code modification to read `motor.cutHeight` and use it as a feature flag.

---

## Things to Try / Investigate

1. **Check RTK link quality:** Is the BLE link between base station and rover solid? Is the base station getting its own good fix? Run `AT+S` at the start and check `sv_dgps` — it should be close to `sv`.

2. **Check `dgps_age_s` during problem moments:** If it spikes, the RTK link is dropping corrections. The rover then degrades from RTK fix → float → invalid, which would cause heading loss.

3. **Consider GPS-only heading mode:** The firmware can derive heading purely from GPS displacement. With no IMU, this is what it does. The question is whether the GPS is giving consistent enough displacement vectors to make this work reliably. A series of `AT+S` readings (10 per minute) during normal operation would show whether `delta` is stable.

4. **Check `gps_jumps` and `gps_chk_err` in `AT+T`:** High values here would indicate GPS signal quality problems, not a software issue.

5. **Re-examine IMU possibility:** Even a noisy or tilted IMU that provides a reasonable heading/yaw rate might help more than nothing. The key question is whether the IMU provides usable *relative* heading changes (gyroscope), not absolute orientation. Gyro drift over a 1-hour mow may be acceptable. The magnetic compass part of the IMU is the questionable piece given motor interference.
