#include "robot/intake.hpp"

#include "robot/config.hpp"
#include "robot/hardware.hpp"

namespace {
void updateIntake() {
	const bool intakeIn = controller.get_digital(intakeInButton);
	const bool intakeOut = controller.get_digital(intakeOutButton);

	if (intakeIn == intakeOut) {
		// Stop when neither direction is held or when both are held.
		intakeMotors.brake();
	} else {
		intakeMotors.move(intakeIn ? intakeSpeed : -intakeSpeed);
	}
}
} // namespace

void intake_in() {
	updateIntake();
}

void intake_out() {
	updateIntake();
}

