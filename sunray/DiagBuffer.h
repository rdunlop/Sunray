// Ardumower Sunray — On-Board Diagnostic Buffer
// Records 1 Hz telemetry snapshots and freezes on fault events.
// Read back after a mow via the AT+DB serial command.

#ifndef DIAG_BUFFER_H
#define DIAG_BUFFER_H

#include <Arduino.h>

#define DIAGBUF_SIZE 120   // entries (120 seconds = 2 minutes)

enum DiagFreezeReason : uint8_t {
  FREEZE_NONE      = 0,
  FREEZE_BLADE_OFF = 1,   // blade motor off for >30 s
  FREEZE_KIDNAPPED = 2,   // SENS_KIDNAPPED triggered
};

// One second of telemetry — 36 bytes with alignment padding
struct DiagEntry {
  uint32_t timestamp_s;    // millis()/1000 at sample time
  float    x;              // robot east position (m)
  float    y;              // robot north position (m)
  float    delta;          // heading (rad)
  float    dgps_age_s;     // RTK correction age (s) — NTRIP dropout indicator
  float    lateral_err;    // lateral tracking error (m)
  float    ground_speed;   // ground speed (m/s) — speed-gate context
  uint8_t  gps_sol;        // 0=invalid, 1=float, 2=fix
  uint8_t  sensor;         // Sensor enum value (e.g. 9=GPS_INVALID, 6=KIDNAPPED)
  uint8_t  op;             // OperationType enum value (0=IDLE, 1=MOW …)
  uint8_t  sv_dgps;        // RTK satellite count
  uint8_t  snaps_fired;    // heading snaps that fired this second
  uint8_t  snaps_blocked;  // heading snaps blocked by speed gate this second
  uint8_t  chk_err;        // GPS NMEA checksum errors this second
  uint8_t  dgps_pkt_s;     // RTCM packets received this second (delta of gps.dgpsPacketCounter)
  uint16_t uart2_rx_Bps;   // bytes/s received on rover F9P UART2 from radio (UBX-MON-COMMS); 0 = radio silent
};

class DiagBuffer {
public:
  // Call once at startup (optional — defaults are safe)
  void begin();

  // Call from the main run loop. Internally gated to 1 Hz.
  void update();

  // Write CSV output to CONSOLE for AT+DB. Does not unfreeze.
  void cmdDump();

  // Unfreeze and clear all entries for AT+DB,R.
  void reset();

  bool isFrozen() const { return frozen; }

private:
  DiagEntry        entries[DIAGBUF_SIZE];
  int              head          = 0;      // next write slot
  int              count         = 0;      // filled entries (0..DIAGBUF_SIZE)
  bool             frozen        = false;
  DiagFreezeReason freezeReason  = FREEZE_NONE;

  unsigned long bladeOffSince       = 0;   // millis() when blade went off, 0 = blade on
  unsigned long nextSampleTime      = 0;   // internal 1 Hz gate

  // Per-second delta tracking (cumulative → delta)
  uint32_t lastHeadingSnaps         = 0;
  uint32_t lastHeadingSnapBlocked   = 0;
  uint32_t lastGpsChkErr            = 0;
  uint32_t lastDgpsPktCount         = 0;
  uint32_t lastUart2RxBytes         = 0;
};

extern DiagBuffer diagBuffer;

#endif
