// Ardumower Sunray — On-Board Diagnostic Buffer
// See DiagBuffer.h for design notes.

#include <Arduino.h>
#include "DiagBuffer.h"
#include "robot.h"
#include "Stats.h"

DiagBuffer diagBuffer;

static const char* freezeReasonStr(DiagFreezeReason r) {
  switch (r) {
    case FREEZE_BLADE_OFF: return "BLADE_OFF_TIMEOUT";
    case FREEZE_KIDNAPPED: return "KIDNAPPED";
    default:               return "NONE";
  }
}

void DiagBuffer::begin() {
  reset();
}

void DiagBuffer::reset() {
  head                   = 0;
  count                  = 0;
  frozen                 = false;
  freezeReason           = FREEZE_NONE;
  bladeOffSince          = 0;
  nextSampleTime         = 0;
  lastHeadingSnaps       = stats.statHeadingSnaps;
  lastHeadingSnapBlocked = stats.statHeadingSnapSpeedBlocked;
  lastGpsChkErr          = gps.chksumErrorCounter;
  lastDgpsPktCount       = gps.dgpsPacketCounter;
  lastUart2RxBytes       = gps.uart2RxBytes;
}

void DiagBuffer::update() {
  if (millis() < nextSampleTime) return;
  nextSampleTime = millis() + 1000;

  // --- Compute per-second deltas from cumulative counters ---
  uint8_t snaps   = (uint8_t)min((uint32_t)255,
                      stats.statHeadingSnaps - lastHeadingSnaps);
  uint8_t blocked = (uint8_t)min((uint32_t)255,
                      stats.statHeadingSnapSpeedBlocked - lastHeadingSnapBlocked);
  uint8_t chkerr  = (uint8_t)min((uint32_t)255,
                      gps.chksumErrorCounter - lastGpsChkErr);
  uint8_t  pkt_s  = (uint8_t)min((uint32_t)255,
                      gps.dgpsPacketCounter - lastDgpsPktCount);
  uint16_t rx_Bps = (uint16_t)min((uint32_t)65535,
                      gps.uart2RxBytes - lastUart2RxBytes);
  lastHeadingSnaps       = stats.statHeadingSnaps;
  lastHeadingSnapBlocked = stats.statHeadingSnapSpeedBlocked;
  lastGpsChkErr          = gps.chksumErrorCounter;
  lastDgpsPktCount       = gps.dgpsPacketCounter;
  lastUart2RxBytes       = gps.uart2RxBytes;

  // --- Record entry if not frozen ---
  if (!frozen) {
    DiagEntry e;
    e.timestamp_s  = millis() / 1000;
    e.x            = stateEstimator.stateX;
    e.y            = stateEstimator.stateY;
    e.delta        = stateEstimator.stateDelta;
    e.dgps_age_s   = (gps.dgpsAge == 0) ? 0.0f
                     : (float)(millis() - gps.dgpsAge) / 1000.0f;
    e.lateral_err  = stateEstimator.lateralError;
    e.ground_speed = stateEstimator.stateGroundSpeed;
    e.gps_sol      = (uint8_t)gps.solution;
    e.sensor       = (uint8_t)stateEstimator.stateSensor;
    e.op           = (uint8_t)stateEstimator.stateOp;
    e.sv_dgps      = (uint8_t)min(gps.numSVdgps, 255);
    e.snaps_fired  = snaps;
    e.snaps_blocked = blocked;
    e.chk_err      = chkerr;
    e.dgps_pkt_s   = pkt_s;
    e.uart2_rx_Bps = rx_Bps;

    entries[head % DIAGBUF_SIZE] = e;
    head = (head + 1) % DIAGBUF_SIZE;
    if (count < DIAGBUF_SIZE) count++;

    // --- Check freeze conditions ---

    // Blade off timeout (>30 s)
    if (motor.pwmMowOut == 0) {
      if (bladeOffSince == 0) bladeOffSince = millis();
      if (millis() - bladeOffSince > 30000UL) {
        frozen       = true;
        freezeReason = FREEZE_BLADE_OFF;
      }
    } else {
      bladeOffSince = 0;  // blade is on — reset timer
    }

    // Kidnap (immediate)
    if (stateEstimator.stateSensor == SENS_KIDNAPPED) {
      frozen       = true;
      freezeReason = FREEZE_KIDNAPPED;
    }
  }
}

void DiagBuffer::cmdDump() {
  CONSOLE.print("DB,reason:");
  CONSOLE.print(freezeReasonStr(freezeReason));
  CONSOLE.print(",entries:");
  CONSOLE.println(count);

  if (count == 0) {
    CONSOLE.println("OK");
    return;
  }

  CONSOLE.println("t_s,x,y,delta_rad,sol,age_s,sensor,op,sv_dgps,lat_err,spd_ms,snaps,blocked,chkerr,dgps_pkt_s,uart2_rx_Bps");

  // Iterate oldest to newest
  int startIdx = (count < DIAGBUF_SIZE) ? 0 : head;
  for (int i = 0; i < count; i++) {
    const DiagEntry& e = entries[(startIdx + i) % DIAGBUF_SIZE];
    CONSOLE.print(e.timestamp_s);    CONSOLE.print(',');
    CONSOLE.print(e.x, 2);          CONSOLE.print(',');
    CONSOLE.print(e.y, 2);          CONSOLE.print(',');
    CONSOLE.print(e.delta, 3);      CONSOLE.print(',');
    CONSOLE.print(e.gps_sol);       CONSOLE.print(',');
    CONSOLE.print(e.dgps_age_s, 2); CONSOLE.print(',');
    CONSOLE.print(e.sensor);        CONSOLE.print(',');
    CONSOLE.print(e.op);            CONSOLE.print(',');
    CONSOLE.print(e.sv_dgps);       CONSOLE.print(',');
    CONSOLE.print(e.lateral_err, 2);CONSOLE.print(',');
    CONSOLE.print(e.ground_speed, 2);CONSOLE.print(',');
    CONSOLE.print(e.snaps_fired);   CONSOLE.print(',');
    CONSOLE.print(e.snaps_blocked); CONSOLE.print(',');
    CONSOLE.print(e.chk_err);      CONSOLE.print(',');
    CONSOLE.print(e.dgps_pkt_s);   CONSOLE.print(',');
    CONSOLE.println(e.uart2_rx_Bps);
  }
  CONSOLE.println("OK");
}
