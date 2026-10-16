#pragma once

#include "pros/misc.h"

// Shared types and every tunable constant for the robot. This header holds no
// state: only types, ports-adjacent settings, and numbers you are expected to
// adjust. The objects themselves are declared in robot/hardware.hpp.

enum class RobotDsrSensor {
	front,
	right,
	back,
	left
};

enum class DsrWall {
	top,
	right,
	bottom,
	left
};

struct DsrSensorConfig {
	float heading_offset; // direction the sensor points compared to robot front, in degrees
	float x_offset; // sensor-local forward offset from tracking point to sensor lens, in inches
	float y_offset; // sensor-local right-side offset from tracking point to sensor lens, in inches

	DsrSensorConfig(float heading_offset, float x_offset, float y_offset)
	    : heading_offset(heading_offset), x_offset(x_offset), y_offset(y_offset) {}
};

// Drivetrain geometry. SINGLE SOURCE OF TRUTH: the lemlib::Drivetrain in
// hardware.cpp, the Ramsete follower, and the simple_* encoder moves all read
// these. Nothing else should hardcode a wheel size or an rpm. These were
// previously written out separately in three places, which is how a 3.25" value
// survived next to a 2.75" one and quietly scaled odometry wrong.
constexpr float kTrackWidthIn = 10.6; // center of middle left wheel to middle right wheel
constexpr float kDriveWheelDiameterIn = 2.75; // matches lemlib::Omniwheel::NEW_275
constexpr float kDriveWheelRpm = 450.0; // wheel rpm after the external gearing
constexpr float kMotorCartridgeRpm = 600.0; // blue cartridge

// TUNE THIS: the track width the Ramsete follower splits a turn rate across to
// get left and right wheel speeds. It starts at the measured kTrackWidthIn, but
// omni wheels scrub sideways through a turn, so the width that actually produces
// the commanded rotation is usually a few percent larger than the measured one.
// Raise it if the robot consistently under-rotates on curves. Odometry does not
// read this, so changing it cannot move the reported pose.
constexpr float kRamseteEffectiveTrackWidthIn = kTrackWidthIn;

// TUNE THIS: the largest linear acceleration the drivetrain can really produce,
// in inches per second squared. The planner exports its velocity column with no
// acceleration limit at all -- the current files step v by up to 675 in/s^2, or
// 17 m/s^2 -- so every trajectory is re-timed against this number as it loads.
// Set it too high and the robot falls behind its own trajectory in the first
// tenth of a second; too low and every path is slower than it needs to be.
// 150 in/s^2 is about 3.8 m/s^2, and costs the current paths 2-3% more time.
constexpr float kRamseteMaxAccelIps2 = 150.0;

// DSR wall locations
constexpr float dsrTopWallY = 72.0; // top wall Y coordinate, in inches
constexpr float dsrRightWallX = 72.0; // right wall X coordinate, in inches
constexpr float dsrBottomWallY = -72.0; // bottom wall Y coordinate, in inches
constexpr float dsrLeftWallX = -72.0; // left wall X coordinate, in inches

// DSR reading limits
constexpr float dsrMinValidDistance = 0.5; // minimum valid distance sensor reading, in inches
constexpr float dsrMaxValidDistance = 100.0; // maximum valid distance sensor reading, in inches

// Main claw driver control.
constexpr auto openClawToggleButton = pros::E_CONTROLLER_DIGITAL_R2;

// Lift travel limits in MOTOR encoder degrees, with the lift lowered at startup.
constexpr double liftLowerLimitDegrees = 0.0;
// CHANGE THIS: 100 degrees is a placeholder. Set the new lift's real upper limit.
constexpr double liftUpperLimitDegrees = 100.0;
constexpr auto liftUpButton = pros::E_CONTROLLER_DIGITAL_L1;
constexpr int liftUpVelocityRpm = 100; // Red cartridges: maximum 100 rpm.
constexpr auto liftDownButton = pros::E_CONTROLLER_DIGITAL_L2;
constexpr int liftDownSlowVelocityRpm = 60; // CHAGE THIS to tune held-button lowering.
constexpr auto liftManualDownButton = pros::E_CONTROLLER_DIGITAL_DOWN;
constexpr int liftManualDownVelocityRpm = 60; // Plain held-button lowering speed.
constexpr unsigned int liftClawOpenWaitMs = 250; // Minimum claw opening time.
constexpr float liftPullAwayDistanceIn = 3.0; // Actual translation before auto-lowering.
constexpr int liftPullAwayJoystickDeadband = 15; // Ignore small throttle-stick inputs.
// CHANGE THIS: fixed clearance above zero whenever the claw is CLOSED.
// Every lowering/reset command respects this floor; it is not a relative nudge.
constexpr double liftClawPickupDegrees = 10.0; // Motor encoder degrees.
// Return on the next manual OPEN click only within twice the pickup amount above the bottom.
constexpr double liftClawReturnMaxDegrees = liftLowerLimitDegrees + 2.0 * liftClawPickupDegrees;
constexpr auto liftResetButton = pros::E_CONTROLLER_DIGITAL_R1; // Return to current floor; does not re-zero encoders.
static_assert(liftUpperLimitDegrees > liftLowerLimitDegrees);
static_assert(liftUpVelocityRpm > 0 && liftUpVelocityRpm <= 100);
static_assert(liftDownSlowVelocityRpm > 0 && liftDownSlowVelocityRpm < liftUpVelocityRpm);
static_assert(liftManualDownVelocityRpm > 0 && liftManualDownVelocityRpm <= 100);
static_assert(liftPullAwayDistanceIn > 0.0);
static_assert(liftClawPickupDegrees > 0.0);
static_assert(liftLowerLimitDegrees + liftClawPickupDegrees < liftUpperLimitDegrees);

// First-pass lift positions for the restored simple auton. CHANGE THESE after
// testing the rebuilt robot; the old 500/350 cascade targets do not transfer.
constexpr double simpleAutonLiftRaisedDegrees = liftUpperLimitDegrees;
constexpr double simpleAutonLiftReleaseDegrees =
    liftLowerLimitDegrees + 0.7 * (liftUpperLimitDegrees - liftLowerLimitDegrees);
constexpr unsigned int simpleAutonLiftTimeoutMs = 2000;
constexpr double liftMoveToleranceDegrees = 2.0;
constexpr unsigned int liftMoveSettleMs = 100;
