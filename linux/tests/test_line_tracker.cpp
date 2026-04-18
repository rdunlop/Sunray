#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <cmath>

#include "test_helpers.h"
#include "LineTracker.h"

// =============================================================================
// Stanley controller — lateral error drives angular correction
// =============================================================================

TEST_CASE("Stanley: robot right of path produces left-turning angular speed") {
    StateEstimator est = defaultEstimator();
    Map            mp  = defaultMap();
    Motor          mot = defaultMotor();
    TestOp         op{};
    Op*            opPtr = &op;

    // Robot at (5, -0.5): 0.5m south of the east-facing path (right-hand side).
    // Positive angular = counterclockwise (left turn); heading east + left turn
    // steers north, back toward the path.  So correction is positive angular.
    est.stateX     = 5.0f;
    est.stateY     = -0.5f;
    est.stateDelta = 0.0f;

    LineTracker lt(makeLineTracker(est, mp, mot, opPtr, SOL_FIXED));
    lt.trackLine(true);

    REQUIRE(mot.angularSpeedSet > 0.0f);
    REQUIRE(mot.linearSpeedSet  > 0.0f);
}

TEST_CASE("Stanley: robot left of path produces right-turning angular speed") {
    StateEstimator est = defaultEstimator();
    Map            mp  = defaultMap();
    Motor          mot = defaultMotor();
    TestOp         op{};
    Op*            opPtr = &op;

    // Robot at (5, +0.5): 0.5m north of the east-facing path (left-hand side).
    // Heading east + right turn (negative angular) steers south, back toward path.
    est.stateX     = 5.0f;
    est.stateY     = 0.5f;
    est.stateDelta = 0.0f;

    LineTracker lt(makeLineTracker(est, mp, mot, opPtr, SOL_FIXED));
    lt.trackLine(true);

    REQUIRE(mot.angularSpeedSet < 0.0f);
    REQUIRE(mot.linearSpeedSet  > 0.0f);
}

TEST_CASE("Stanley: robot on path with aligned heading produces near-zero angular speed") {
    StateEstimator est = defaultEstimator();
    Map            mp  = defaultMap();
    Motor          mot = defaultMotor();
    TestOp         op{};
    Op*            opPtr = &op;

    est.stateX     = 5.0f;
    est.stateY     = 0.0f;  // exactly on path
    est.stateDelta = 0.0f;  // heading exactly east

    LineTracker lt(makeLineTracker(est, mp, mot, opPtr, SOL_FIXED));
    lt.trackLine(true);

    REQUIRE(std::fabs(mot.angularSpeedSet) < 0.05f);
    REQUIRE(mot.linearSpeedSet > 0.0f);
}

// =============================================================================
// Angle alignment — large misalignment triggers rotation-only
// =============================================================================

TEST_CASE("Angle alignment: 90-degree misalignment produces rotation with no forward speed") {
    StateEstimator est = defaultEstimator();
    Map            mp  = defaultMap();
    Motor          mot = defaultMotor();
    TestOp         op{};
    Op*            opPtr = &op;

    // Robot on path but heading south — 90° off the east-facing path.
    // TARGET_ANGLE_TOLERANCE = 20°, so this triggers pure rotation.
    est.stateX     = 5.0f;
    est.stateY     = 0.0f;
    est.stateDelta = -(float)M_PI / 2.0f;  // heading south

    LineTracker lt(makeLineTracker(est, mp, mot, opPtr, SOL_FIXED));
    lt.trackLine(true);

    REQUIRE(std::fabs(mot.linearSpeedSet)  < 0.01f);
    REQUIRE(std::fabs(mot.angularSpeedSet) > 0.0f);
}

// =============================================================================
// Speed hierarchy — GPS solution caps speed
// =============================================================================

TEST_CASE("Speed cap: GPS float solution limits linear speed to 0.1 m/s") {
    StateEstimator est = defaultEstimator();
    Map            mp  = defaultMap();
    Motor          mot = defaultMotor();
    TestOp         op{};
    Op*            opPtr = &op;

    est.stateX     = 5.0f;
    est.stateY     = 0.0f;
    est.stateDelta = 0.0f;
    est.setSpeed   = 0.5f;

    LineTracker lt(makeLineTracker(est, mp, mot, opPtr, SOL_FLOAT));
    lt.trackLine(true);

    REQUIRE(std::fabs(mot.linearSpeedSet) <= 0.1f + 1e-4f);
}

// =============================================================================
// Speed hierarchy — motor overload caps speed
// =============================================================================

TEST_CASE("Speed cap: motor left overload limits linear speed to 0.1 m/s") {
    StateEstimator est = defaultEstimator();
    Map            mp  = defaultMap();
    Motor          mot = defaultMotor();
    TestOp         op{};
    Op*            opPtr = &op;

    est.stateX     = 5.0f;
    est.stateY     = 0.0f;
    est.stateDelta = 0.0f;
    est.setSpeed   = 0.5f;
    mot.motorLeftOverload = true;

    LineTracker lt(makeLineTracker(est, mp, mot, opPtr, SOL_FIXED));
    lt.trackLine(true);

    // MOTOR_OVERLOAD_SPEED = 0.1 m/s
    REQUIRE(std::fabs(mot.linearSpeedSet) <= 0.1f + 1e-4f);
}

// =============================================================================
// Kidnap detection
// =============================================================================

TEST_CASE("Kidnap: cross-track error > 1.0m fires onKidnapped(true)") {
    StateEstimator est = defaultEstimator();
    Map            mp  = defaultMap();
    Motor          mot = defaultMotor();
    TestOp         op{};
    Op*            opPtr = &op;

    // 2m north of path — beyond the 1.0m KIDNAP_DETECT_ALLOWED_PATH_TOLERANCE
    est.stateX                = 5.0f;
    est.stateY                = 2.0f;
    est.stateDelta            = 0.0f;
    est.stateLocalizationMode = LOC_GPS;

    LineTracker lt(makeLineTracker(est, mp, mot, opPtr, SOL_FIXED));
    lt.trackLine(false);  // runControl=false: check callbacks without writing motor

    REQUIRE(op.onKidnappedCount   == 1);
    REQUIRE(op.lastKidnappedState == true);
}

TEST_CASE("Kidnap: returning to path after kidnap fires onKidnapped(false)") {
    StateEstimator est = defaultEstimator();
    Map            mp  = defaultMap();
    Motor          mot = defaultMotor();
    TestOp         op{};
    Op*            opPtr = &op;

    est.stateLocalizationMode = LOC_GPS;
    est.stateDelta            = 0.0f;

    LineTracker lt(makeLineTracker(est, mp, mot, opPtr, SOL_FIXED));

    // First call: trigger kidnap (2m off path)
    est.stateX = 5.0f;
    est.stateY = 2.0f;
    lt.trackLine(false);
    REQUIRE(op.lastKidnappedState == true);

    // Second call: back on path
    est.stateY = 0.0f;
    lt.trackLine(false);

    REQUIRE(op.onKidnappedCount   == 2);
    REQUIRE(op.lastKidnappedState == false);
}

// =============================================================================
// Target reached
// =============================================================================

TEST_CASE("Target reached: robot within tolerance fires onTargetReached") {
    StateEstimator est = defaultEstimator();
    Map            mp  = defaultMap();
    Motor          mot = defaultMotor();
    TestOp         op{};
    Op*            opPtr = &op;

    // Target at (5, 0). Robot is 5 cm past it — within TARGET_REACHED_TOLERANCE (0.1m).
    mp.lastTargetPoint.setXY(0.0f, 0.0f);
    mp.targetPoint.setXY(5.0f,     0.0f);
    est.stateX     = 5.0f + 0.05f;
    est.stateY     = 0.0f;
    est.stateDelta = 0.0f;

    LineTracker lt(makeLineTracker(est, mp, mot, opPtr, SOL_FIXED));
    lt.trackLine(false);

    REQUIRE(op.onTargetReachedCount >= 1);
}
