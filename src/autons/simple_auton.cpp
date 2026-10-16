#include "autons/autons.hpp"

#include "22020Vlib/drive.hpp"
#include "robot/hardware.hpp"
#include "robot/lift.hpp"
#include "pros/llemu.hpp"
#include "pros/rtos.hpp"
#include <cmath>

namespace {
bool moveLift(double targetDegrees, int velocityRpm) {
	if (lift_move_to(targetDegrees, velocityRpm, simpleAutonLiftTimeoutMs)) return true;
	// Abort the remaining routine if the new lift stalls or loses feedback.
	leftMotors.brake();
	rightMotors.brake();
	pros::lcd::set_text(4, "Auton: lift move failed");
	return false;
}

bool clearedWall() {
	// simple_drive_distance() zeroes these first-motor encoders and selects
	// degrees. Check the completed 20-inch move directly; this old routine's
	// encoder helpers do not preserve an odometry reference for pull-away checks.
	const double leftDegrees = std::fabs(leftMotors.get_position(0));
	const double rightDegrees = std::fabs(rightMotors.get_position(0));
	if (!std::isfinite(leftDegrees) || !std::isfinite(rightDegrees)) return false;
	constexpr double pi = 3.141592653589793;
	const double distanceInches = (leftDegrees + rightDegrees) / 2.0 / 360.0 *
	    (kDriveWheelRpm / kMotorCartridgeRpm) * pi * kDriveWheelDiameterIn;
	return distanceInches >= liftPullAwayDistanceIn;
}
} // namespace

void simple_auton() {
	if (pros::competition::is_disabled()) return;
	pros::lcd::set_text(4, "Auton: simple lift");
	OpenClaw.retract(); // Secure the preload; never depend on toggle history.

	// Replace the old arm/cascade setup with clearance, then the raised lift.
	if (!moveLift(liftLowerLimitDegrees + liftClawPickupDegrees, liftUpVelocityRpm)) return;
	if (!moveLift(simpleAutonLiftRaisedDegrees, liftUpVelocityRpm)) return;

	// Original drivetrain sequence. Distances are inches; turn angles are degrees.
	constexpr double backwardDistanceInches = -12.0;
	constexpr double forwardDistanceInches = 15.0;
	constexpr std::int32_t drivePower = 60;
	for (int movement = 0; movement < 2; movement++) {
		simple_drive_distance(backwardDistanceInches, drivePower);
		simple_drive_distance(forwardDistanceInches, drivePower);
	}
	simple_drive_distance(-16.0, drivePower);
	simple_turn_degrees(-110.0, 50);
	simple_drive_backward_until_wall(50, 2000);

	// Replace cascade 350 with a tunable placement height, then release once.
	if (!moveLift(simpleAutonLiftReleaseDegrees, liftDownSlowVelocityRpm)) return;
	OpenClaw.extend();
	pros::delay(liftClawOpenWaitMs);

	// Keep the lift held during the original turn and drive away from the score.
	simple_turn_degrees(200.0, 60);
	simple_drive_distance(20.0, 60);
	if (!clearedWall()) {
		pros::lcd::set_text(4, "Auton: no pull-away");
		return; // Stay raised instead of lowering while still against the score.
	}
	if (!moveLift(liftLowerLimitDegrees, liftUpVelocityRpm)) return;
	simple_turn_degrees(-100.0, 60);
	pros::lcd::set_text(4, "Auton: complete");
}
