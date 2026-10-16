#include "autons/autons.hpp"

#include "22020Vlib/drive.hpp"
#include "robot/hardware.hpp"
#include "robot/lift.hpp"
#include "pros/llemu.hpp"
#include "pros/rtos.hpp"

namespace {
bool moveLift(double targetDegrees, int velocityRpm) {
	if (lift_move_to(targetDegrees, velocityRpm, simpleAutonLiftTimeoutMs)) return true;
	// Abort the remaining routine if the new lift stalls or loses feedback.
	leftMotors.brake();
	rightMotors.brake();
	pros::lcd::set_text(4, "Auton: lift move failed");
	return false;
}

bool checkDrive(bool completed) {
	if (completed) return true;
	leftMotors.brake();
	rightMotors.brake();
	liftMotors.brake();
	pros::lcd::set_text(4, "Auton: drive move failed");
	return false;
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
		if (!checkDrive(simple_drive_distance(backwardDistanceInches, drivePower))) return;
		if (!checkDrive(simple_drive_distance(forwardDistanceInches, drivePower))) return;
	}
	if (!checkDrive(simple_drive_distance(-16.0, drivePower))) return;
	if (!checkDrive(simple_turn_degrees(-110.0, 50))) return;
	if (!checkDrive(simple_drive_backward_until_wall(50, 2000))) return;

	// Replace cascade 350 with a tunable placement height, then release once.
	if (!moveLift(simpleAutonLiftReleaseDegrees, liftDownSlowVelocityRpm)) return;
	OpenClaw.extend();
	pros::delay(liftClawOpenWaitMs);

	// Keep the lift held during the original turn and drive away from the score.
	if (!checkDrive(simple_turn_degrees(200.0, 60))) return;
	// Require the full drive-away to finish before lowering. A stalled drive
	// leaves the lift held at placement height and aborts the remaining routine.
	if (!checkDrive(simple_drive_distance(20.0, 60))) return;
	if (!moveLift(liftLowerLimitDegrees, liftUpVelocityRpm)) return;
	if (!checkDrive(simple_turn_degrees(-100.0, 60))) return;
	pros::lcd::set_text(4, "Auton: complete");
}
