#include "autons/autons.hpp"

#include "22020Vlib/drive.hpp"
#include "robot/claw.hpp"
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

} // namespace

void simple_auton() {
	if (pros::competition::is_disabled()) return;
	pros::lcd::set_text(4, "Auton: simple lift");
	OpenClaw.retract(); // Secure the preload; never depend on toggle history.
	// Lift-only test positions from the updated auton. lift_move_to clamps these
	// to the configured travel range and closed-claw minimum.
	if (!moveLift(200.0, 60)) return;
	if (!moveLift(0.0, 60)) return;
	// The supplied updated file declares -8 and +5 inch drive values but has no
	// drive calls, so this routine remains a lift-only test.
	pros::lcd::set_text(4, "Auton: complete");
}
