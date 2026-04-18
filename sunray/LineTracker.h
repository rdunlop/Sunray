// Ardumower Sunray
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)


#ifndef LINE_TRACKER_H
#define LINE_TRACKER_H


#include <functional>  // Must precede Arduino.h: Arduino defines min/max as 2-arg macros
                       // that corrupt std::min/max 3-arg overloads in <bits/stl_algobase.h>.
#include <Arduino.h>
#include "config.h"

// Forward declarations (full headers included in LineTracker.cpp)
class StateEstimator;
class Map;
class Motor;
class Op;

// gps.h only defines the SolType enum — no heavy dependencies
#include "gps.h"


class LineTracker {
public:
  // Stanley controller gains (exposed for tuning)
  float stanleyTrackingNormalK = STANLEY_CONTROL_K_NORMAL;
  float stanleyTrackingNormalP = STANLEY_CONTROL_P_NORMAL;
  float stanleyTrackingSlowK = STANLEY_CONTROL_K_SLOW;
  float stanleyTrackingSlowP = STANLEY_CONTROL_P_SLOW;

  LineTracker();  // production: pointers wired to firmware globals at construction
  LineTracker(StateEstimator& est, Map& mp, Motor& mot, Op*& op,
              std::function<SolType()> gpsSol,
              std::function<unsigned long()> millisFn = nullptr);  // testing: injected deps

  void trackLine(bool runControl);

private:
  bool rotateLeft = false;
  bool rotateRight = false;
  bool angleToTargetFits = false;
  bool langleToTargetFits = false;
  bool targetReached = false;
  float trackerDiffDelta = 0;
  bool stateKidnapped = false;
  bool printmotoroverload = false;
  bool trackerDiffDelta_positive = false;
  float lastLineDist = 0;
  unsigned long gpsDegradedSince = 0;  // millis() when GPS first dropped below SOL_FIXED; 0 = OK

  StateEstimator*          _est;
  Map*                     _map;
  Motor*                   _mot;
  Op**                     _op;   // pointer-to-pointer so activeOp changes are visible
  std::function<SolType()>      _gpsSol;
  std::function<unsigned long()> _millisFn;  // null → real millis()
};



#endif
