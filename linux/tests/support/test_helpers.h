#pragma once

// Pull in the globals and types we need. op.h → robot.h → all extern globals.
#include "op/op.h"
#include "LineTracker.h"
#include "gps.h"
#include "StateEstimator.h"
#include "map.h"
#include "motor.h"

// ---------------------------------------------------------------------------
// TestOp
//
// Minimal Op subclass that records which event callbacks LineTracker fired.
// All Op base-class virtual methods have default no-op implementations, so we
// only override the ones we want to observe.
// ---------------------------------------------------------------------------
class TestOp : public Op {
public:
    int  onTargetReachedCount       = 0;
    int  onNoFurtherWaypointsCount  = 0;
    int  onKidnappedCount           = 0;
    bool lastKidnappedState         = false;
    int  onGpsFixTimeoutCount       = 0;
    int  onGpsNoSignalCount         = 0;
    int  onObstacleCount            = 0;

    String name() override { return "TestOp"; }

    void onTargetReached()      override { ++onTargetReachedCount; }
    void onNoFurtherWaypoints() override { ++onNoFurtherWaypointsCount; }
    void onKidnapped(bool s)    override { ++onKidnappedCount; lastKidnappedState = s; }
    void onGpsFixTimeout()      override { ++onGpsFixTimeoutCount; }
    void onGpsNoSignal()        override { ++onGpsNoSignalCount; }
    void onObstacle()           override { ++onObstacleCount; }

    // Prevent changeOp() from trying to transition to real operations
    void changeOp(Op&, bool = false) override {}
};

// ---------------------------------------------------------------------------
// makeLineTracker
//
// Construct a LineTracker wired to the provided local objects.
// `sol` is captured by value so each test gets a fixed GPS solution.
//
// Usage:
//   StateEstimator est = defaultEstimator();
//   Map            mp  = defaultMap();
//   Motor          mot = defaultMotor();
//   TestOp         op{};
//   Op*            opPtr = &op;
//
//   LineTracker lt = makeLineTracker(est, mp, mot, opPtr, SOL_FIXED);
// ---------------------------------------------------------------------------
inline LineTracker makeLineTracker(
    StateEstimator& est,
    Map&            mp,
    Motor&          mot,
    Op*&            opPtr,
    SolType         sol,
    std::function<unsigned long()> millisFn = nullptr)
{
    return LineTracker(est, mp, mot, opPtr, [sol]{ return sol; }, millisFn);
}

// ---------------------------------------------------------------------------
// defaultEstimator / defaultMap / defaultMotor
//
// Convenience functions that return objects pre-set to a sensible test
// baseline: robot at origin, heading east (delta=0), on a straight east-
// facing path from (0,0) to (10,0), GPS fixed, no overloads.
// ---------------------------------------------------------------------------

inline StateEstimator defaultEstimator() {
    StateEstimator est{};
    est.stateX                 = 5.0f;   // midway along path
    est.stateY                 = 0.0f;
    est.stateDelta             = 0.0f;   // heading east
    est.stateGroundSpeed       = 0.5f;
    est.setSpeed               = 0.5f;
    est.stateLocalizationMode  = LOC_GPS;
    est.linearMotionStartTime  = 0;
    est.angularMotionStartTime = 0;
    est.fixTimeout             = 0;      // disable GPS fix-timeout path
    est.lateralError           = 0.0f;
    est.stateAprilTagFound     = false;
    est.stateReflectorTagFound = false;
    return est;
}

inline Map defaultMap() {
    Map mp{};
    mp.lastTargetPoint.setXY(0.0f,  0.0f);
    mp.targetPoint.setXY(10.0f,     0.0f);
    mp.trackReverse = false;
    mp.trackSlow    = false;
    mp.wayMode      = WAY_MOW;
    return mp;
}

inline Motor defaultMotor() {
    Motor mot{};
    mot.motorLeftOverload  = false;
    mot.motorRightOverload = false;
    mot.motorMowOverload   = false;
    mot.motorMowSpinUpTime = 0;
    mot.linearSpeedSet     = 0.0f;
    mot.angularSpeedSet    = 0.0f;
    return mot;
}
