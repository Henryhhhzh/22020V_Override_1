#include "robot/claw.hpp"
#include "robot/config.hpp"
#include "robot/hardware.hpp"
#include "robot/lift.hpp"

void toggleOpenClaw() {
	OpenClaw.toggle();
}

void tiltClawForIntake() {
	ClawTilt.retract();
}

void restoreClawTilt() {
	ClawTilt.extend();
}

void handleClawControl() {
	if (controller.get_digital_new_press(openClawToggleButton)) {
		toggleOpenClaw();
		requestLiftPickupAssist(!OpenClaw.is_extended());
	}
}
