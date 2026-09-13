#include "main.h"
#include "autons/autons.hpp"
#include "robot/claw.hpp"
#include "robot/hardware.hpp"
#include <atomic>
#include <cmath>

// The mechanism controller owns both motors continuously. Targets are stored
// in centidegrees so changing a target is atomic on the V5 brain.
std::atomic<std::int32_t> armTargetCentidegrees{0};
std::atomic<std::int32_t> cascadeTargetCentidegrees{0};
std::atomic<std::int32_t> armManualPower{0};
std::atomic<bool> armManualCoast{false};
std::atomic<std::int32_t> cascadeManualPower{0};

void mechanismPositionController();
std::int32_t degreesToCentidegrees(double degrees);


/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	// Nothing that reads chassis.getPose() works until this runs: it calibrates
	// the IMU, resets the tracking wheels, and starts LemLib's odometry task.
	// With both vertical slots left null it also builds the forward tracking
	// wheels from the drive motor groups. Blocks for roughly three seconds, and
	// the robot must be still for all of it.
	chassis.calibrate();

	// The physical startup pose is the sensor origin: arm 0, cascade 0.
	// The controller then moves both mechanisms to startupBasePosition.
	// Targets are not wrapped, so values such as 400 degrees remain valid.
	cascadeMotor.set_encoder_units(pros::MotorUnits::degrees);
	cascadeMotor.tare_position();
	armRotationSensor.reset_position();
	cascadeMotor.set_brake_mode(cascadeHoldingEnabled ? pros::MotorBrake::hold
	                                                : pros::MotorBrake::coast);
	armMotor.set_brake_mode(armHoldingEnabled ? pros::MotorBrake::hold
	                                        : pros::MotorBrake::coast);
	// The main claw's retracted state is its closed starting state.
	OpenClaw.retract();
	armTargetCentidegrees.store(degreesToCentidegrees(startupBasePosition.armDegrees));
	cascadeTargetCentidegrees.store(degreesToCentidegrees(startupBasePosition.cascadeDegrees));
	armManualPower.store(0);
	armManualCoast.store(false);
	cascadeManualPower.store(0);

	// This task keeps using the external arm sensor after a move has arrived, so
	// gravity or gearbox backlash cannot turn the PID off after a fixed timeout.
	static pros::Task mechanismControllerTask(mechanismPositionController, "Mechanism PID");
	(void)mechanismControllerTask;

	pros::lcd::register_btn1_cb(on_center_button);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

//robot functions driver and auton

// Show continuous mechanism positions on the handheld controller for
// calibration without writing over the V5 brain's PROS/LLEMU interface.
// Only one controller line is sent per update to respect its slower link.
void displayMechanismDegrees() {
	static std::uint32_t lastUpdateMs = 0;
	static bool updateArmLine = true;
	const std::uint32_t now = pros::millis();
	if (now - lastUpdateMs < 100) return;
	lastUpdateMs = now;

	const std::int32_t armCentidegrees = armRotationSensor.get_position();
	const double cascadeDegrees = cascadeMotor.get_position();

	if (updateArmLine) {
		if (armCentidegrees == PROS_ERR) {
			controller.print(0, 0, "Arm: ERROR     ");
		} else {
			controller.print(0, 0, "Arm:%8.2f", armCentidegrees / 100.0);
		}
	} else {
		if (!std::isfinite(cascadeDegrees)) {
			controller.print(1, 0, "Cascade: ERROR ");
		} else {
			controller.print(1, 0, "Cascade:%6.1f", cascadeDegrees);
		}
	}

	updateArmLine = !updateArmLine;
}

std::int32_t degreesToCentidegrees(double degrees) {
	return static_cast<std::int32_t>(std::round(degrees * 100.0));
}

void setArmTarget(double degrees) {
	degrees = std::fmax(armMinimumDegrees, std::fmin(armMaximumDegrees, degrees));
	armTargetCentidegrees.store(degreesToCentidegrees(degrees));
}

void setCascadeTarget(double degrees) {
	degrees = std::fmax(cascadeMinimumDegrees, std::fmin(cascadeMaximumDegrees, degrees));
	cascadeTargetCentidegrees.store(degreesToCentidegrees(degrees));
}

// Full power outside the slowdown range, then a linear approach to the upward
// holding power. The holding power replaces the zero at the end of a normal
// 100-to-0 profile because gravity would make the mechanism sag at zero power.
double linearMechanismOutput(double errorDegrees, double slowdownDegrees, double holdPower) {
	constexpr double maxPower = 127.0;
	double normalizedError = errorDegrees / slowdownDegrees;
	normalizedError = std::fmax(-1.0, std::fmin(1.0, normalizedError));

	if (normalizedError >= 0) {
		return holdPower + (maxPower - holdPower) * normalizedError;
	}
	return holdPower + (maxPower + holdPower) * normalizedError;
}

// Smooth S-curve for the arm. Its gentle slopes remove the high-power approach
// that was carrying the arm past its target through gearbox backlash.
double curvedArmOutput(double errorDegrees, double slowdownDegrees, double holdPower,
                       double maximumPower, double downwardCorrectionPerDegree) {
	double normalizedError = errorDegrees / slowdownDegrees;
	normalizedError = std::fmax(-1.0, std::fmin(1.0, normalizedError));
	const double magnitude = std::abs(normalizedError);
	const double curvedMagnitude = magnitude * magnitude * (3.0 - 2.0 * magnitude);

	if (normalizedError >= 0) {
		return holdPower + (maximumPower - holdPower) * curvedMagnitude;
	}

	// Use a stronger linear correction above the target. Applying the same broad
	// S-curve here allowed the gravity feedforward to keep driving upward even
	// tens of degrees too high (for example, +26 power at a -24 degree error).
	return std::fmax(-maximumPower,
	                 holdPower + downwardCorrectionPerDegree * errorDegrees);
}

// This is the only task that commands the arm and cascade motors. It never
// times out: after reaching a target, it keeps correcting both mechanisms.
void mechanismPositionController() {
	// Increase a hold power if that mechanism still rests below its target;
	// decrease it if the mechanism steadily creeps upward.
	constexpr double armUpwardHoldPower = 15.0;
	constexpr double armMinimumUpwardMovePower = 60.0;
	constexpr double armMovePowerFadeDegrees = 12.0;
	constexpr double armMaximumMovePower = 85.0;
	constexpr double armDownwardCorrectionPerDegree = 4.0;
	constexpr double armVelocityFilterWeight = 0.20;
	constexpr double armVelocityDamping = 0.45;
	constexpr double armDampingRangeDegrees = 35.0;
	constexpr double armMaximumDampingPower = 35.0;
	constexpr double cascadeUpwardHoldPower = 25.0;

	// The arm uses its curve across most of a normal move for gentler motion.
	// The cascade keeps its separate linear slowdown behavior.
	constexpr double armSlowdownDegrees = 100.0;
	constexpr double cascadeSlowdownDegrees = 150.0;
	constexpr double fullPowerKickMinimumErrorDegrees = 1.0;

	std::int32_t armPreviousTarget = 0;
	std::int32_t armPreviousPosition = 0;
	std::uint32_t armPreviousSampleTime = 0;
	double armFilteredVelocity = 0.0;
	bool armPidInitialized = false;

	std::int32_t cascadePreviousTarget = 0;
	bool cascadePidInitialized = false;

	while (true) {
		if (pros::competition::is_disabled()) {
			armMotor.move(0);
			cascadeMotor.move(0);
			armPidInitialized = false;
			cascadePidInitialized = false;
			pros::delay(mechanismLoopMs);
			continue;
		}

		const std::int32_t manualArmPower = armManualPower.load();
		const bool manualArmCoast = armManualCoast.load();
		const std::int32_t armTarget = armTargetCentidegrees.load();
		const std::int32_t armPosition = armRotationSensor.get_position();
		if (armPosition == PROS_ERR) {
			armMotor.move(0);
			armPidInitialized = false;
		} else if (manualArmCoast) {
			// D-pad left releases the arm under gravity instead of powering it down.
			// Restore braking at 0 degrees so coast cannot pass the lower safety limit.
			if (armPosition <= armMinimumCentidegrees) {
				armMotor.set_brake_mode(pros::MotorBrake::hold);
			}
			armMotor.move(0);
			armPidInitialized = false;
		} else if (manualArmPower != 0) {
			const bool pushingPastUpperLimit =
			    manualArmPower > 0 && armPosition >= armMaximumCentidegrees;
			const bool pushingPastLowerLimit =
			    manualArmPower < 0 && armPosition <= armMinimumCentidegrees;
			armMotor.move(pushingPastUpperLimit || pushingPastLowerLimit ? 0 : manualArmPower);
			armPidInitialized = false;
		} else if (!armHoldingEnabled) {
			// With no manual input, send zero power and let the coast brake mode act.
			armMotor.move(0);
			armPidInitialized = false;
		} else if (armTarget <= armMinimumCentidegrees &&
		           armPosition <= armMinimumCentidegrees + armZeroRestBandCentidegrees) {
			// At the bottom, rely on HOLD braking instead of reacting to tiny negative
			// readings and repeatedly triggering the upward movement-power floor.
			armMotor.move(0);
			armPidInitialized = false;
		} else {
			const double armError = (armTarget - armPosition) / 100.0;
			const bool targetChanged = !armPidInitialized || armTarget != armPreviousTarget;
			const std::uint32_t armSampleTime = pros::millis();
			if (targetChanged) {
				armPreviousTarget = armTarget;
				armPreviousPosition = armPosition;
				armPreviousSampleTime = armSampleTime;
				armFilteredVelocity = 0.0;
				armPidInitialized = true;
			} else {
				const std::uint32_t sampleTimeMs = armSampleTime - armPreviousSampleTime;
				if (sampleTimeMs > 0) {
					const double rawVelocity =
					    ((armPosition - armPreviousPosition) / 100.0) *
					    (1000.0 / sampleTimeMs);
					armFilteredVelocity +=
					    armVelocityFilterWeight * (rawVelocity - armFilteredVelocity);
					armPreviousPosition = armPosition;
					armPreviousSampleTime = armSampleTime;
				}
			}

			// Position zero receives no upward feedforward, so position 1 can rest at
			// the bottom. Every raised target gets gravity compensation.
			const double armHoldPower = armTarget > 0 ? armUpwardHoldPower : 0.0;
			double armOutput =
			    curvedArmOutput(armError, armSlowdownDegrees, armHoldPower,
			                    armMaximumMovePower, armDownwardCorrectionPerDegree);
			// A heavy arm needs more torque to begin lifting than it needs to remain
			// stationary. Apply a separate movement floor only while below target,
			// then fade it into the lower hold power over the final few degrees.
			if (armError > 0.0) {
				const double movePowerFraction =
				    std::fmin(1.0, armError / armMovePowerFadeDegrees);
				const double minimumUpwardOutput =
				    armHoldPower +
				    (armMinimumUpwardMovePower - armHoldPower) * movePowerFraction;
				armOutput = std::fmax(armOutput, minimumUpwardOutput);
			}
			// Remove power in proportion to measured arm speed as it approaches the
			// target. If the arm stalls, velocity becomes zero and the full movement
			// floor automatically returns, which is important for this heavy arm.
			const double dampingFraction = std::fmax(
			    0.0, 1.0 - std::abs(armError) / armDampingRangeDegrees);
			double dampingPower = armVelocityDamping * armFilteredVelocity * dampingFraction;
			dampingPower = std::fmax(-armMaximumDampingPower,
			                         std::fmin(armMaximumDampingPower, dampingPower));
			armOutput -= dampingPower;
			// Automatic arm movement is intentionally capped below full power.
			armOutput = std::fmax(-127.0, std::fmin(127.0, armOutput));
			armMotor.move(static_cast<std::int32_t>(std::round(armOutput)));
		}

		const std::int32_t manualCascadePower = cascadeManualPower.load();
		const std::int32_t cascadeTarget = cascadeTargetCentidegrees.load();
		const double cascadePosition = cascadeMotor.get_position();
		if (!std::isfinite(cascadePosition)) {
			cascadeMotor.move(0);
			cascadePidInitialized = false;
		} else if (manualCascadePower != 0) {
			// Manual buttons temporarily own the cascade. Resetting this flag makes
			// the position controller restart cleanly when the buttons are released.
			const bool pushingPastUpperLimit =
			    manualCascadePower > 0 && cascadePosition >= cascadeMaximumDegrees;
			const bool pushingPastLowerLimit =
			    manualCascadePower < 0 && cascadePosition <= cascadeMinimumDegrees;
			cascadeMotor.move(pushingPastUpperLimit || pushingPastLowerLimit
			                      ? 0
			                      : manualCascadePower);
			cascadePidInitialized = false;
		} else if (!cascadeHoldingEnabled) {
			// With no manual input, send zero power and let the coast brake mode act.
			cascadeMotor.move(0);
			cascadePidInitialized = false;
		} else {
			const double cascadeTargetDegrees = cascadeTarget / 100.0;
			const double cascadeError = cascadeTargetDegrees - cascadePosition;
			const bool targetChanged =
			    !cascadePidInitialized || cascadeTarget != cascadePreviousTarget;
			if (targetChanged) {
				cascadePreviousTarget = cascadeTarget;
				cascadePidInitialized = true;
			}

			const double cascadeHoldPower =
			    cascadeTarget > 0 ? cascadeUpwardHoldPower : 0.0;
			double cascadeOutput =
			    linearMechanismOutput(cascadeError, cascadeSlowdownDegrees, cascadeHoldPower);
			if (targetChanged &&
			    std::abs(cascadeError) > fullPowerKickMinimumErrorDegrees) {
				cascadeOutput = std::copysign(127.0, cascadeError);
			}
			cascadeOutput = std::fmax(-127.0, std::fmin(127.0, cascadeOutput));
			cascadeMotor.move(static_cast<std::int32_t>(std::round(cascadeOutput)));
		}

		pros::delay(mechanismLoopMs);
	}
}

// Set an absolute cascade target and wait until it remains within 25 degrees.
// During calibration, correction is disabled, so this records the target and
// returns without trying to move or hold the cascade.
void cascadeSpinToDegree(double degrees) {
	setCascadeTarget(degrees);
	if (!cascadeHoldingEnabled) return;

	const std::int32_t requestedTarget = cascadeTargetCentidegrees.load();
	const double requestedDegrees = requestedTarget / 100.0;
	std::uint32_t settledSince = 0;

	while (cascadeTargetCentidegrees.load() == requestedTarget) {
		const double position = cascadeMotor.get_position();
		if (!std::isfinite(position)) return;

		const double error = requestedDegrees - position;
		if (std::abs(error) <= cascadeTargetToleranceDegrees) {
			if (settledSince == 0) settledSince = pros::millis();
			if (pros::millis() - settledSince >= mechanismSettleTimeMs) return;
		} else {
			settledSince = 0;
		}
		pros::delay(mechanismLoopMs);
	}
}

// Set an absolute arm target and wait until the port-5 Rotation Sensor settles.
// A timeout of 0 keeps the original unlimited wait behavior.
void armSpinToDegree(double degrees, std::uint32_t timeoutMs = 0) {
	setArmTarget(degrees);
	if (!armHoldingEnabled) return;

	const std::int32_t requestedTarget = armTargetCentidegrees.load();
	const std::uint32_t movementStart = pros::millis();
	std::uint32_t settledSince = 0;

	while (armTargetCentidegrees.load() == requestedTarget) {
		const std::int32_t position = armRotationSensor.get_position();
		if (position == PROS_ERR) return;
		if (timeoutMs > 0 && pros::millis() - movementStart >= timeoutMs) {
			// Cancel the old target and hold wherever the arm reached.
			setArmTarget(position / 100.0);
			return;
		}

		if (std::abs(requestedTarget - position) <= armTargetToleranceCentidegrees) {
			if (settledSince == 0) settledSince = pros::millis();
			if (pros::millis() - settledSince >= mechanismSettleTimeMs) return;
		} else {
			settledSince = 0;
		}
		pros::delay(mechanismLoopMs);
	}
}

// The selector is one-based to match the position names above.
int selectedPosition = 1;

void moveToPosition(int positionNumber) {
	if (positionNumber < 1 || positionNumber > 6) return;

	const PositionTargets& target = positions[positionNumber - 1];
	// Driver presets only update the persistent targets, so the chassis loop
	// remains responsive while both mechanisms move and hold simultaneously.
	setArmTarget(target.armDegrees);
	setCascadeTarget(target.cascadeDegrees);
}

// Advance by exactly one preset per call. This intentionally saturates at
// position 6 instead of wrapping back to position 1.
void togglePosition() {
	if (selectedPosition >= 6) return;

	selectedPosition++;
	moveToPosition(selectedPosition);
}

// Select and move directly to position 1 from any current preset.
void goToPositionOne() {
	selectedPosition = 1;
	moveToPosition(1);
}

void toggleOpenClaw() {
	OpenClaw.toggle();

	// Logical retracted means physically closed for this inverted solenoid.
	// Each time it closes, raise the cascade 75 degrees from its current position.
	if (!OpenClaw.is_extended()) {
		const double currentCascadePosition = cascadeMotor.get_position();
		if (std::isfinite(currentCascadePosition)) {
			setCascadeTarget(currentCascadePosition + clawCloseCascadeLiftDegrees);
		}
	}
}

void toggleVariableClaw() {
	VariableClaw.toggle();
}

// Run outward only while controller B is held; otherwise stop the motor.
void handleOuttakeControl() {
	intakeMotor.move(controller.get_digital(outtakeButton) ? outtakeSpeed : 0);
}

// L2 actively raises the arm. D-pad left switches it to coast so gravity can
// lower it without negative motor power. Releasing captures and holds its angle.
void handleManualArmControl() {
	const bool upHeld = controller.get_digital(armManualUpButton);
	const bool downHeld = controller.get_digital(armManualDownButton);
	const std::int32_t requestedPower = upHeld && !downHeld ? armManualSpeed : 0;
	const bool requestedCoast = downHeld && !upHeld;

	const std::int32_t previousPower = armManualPower.load();
	const bool previouslyCoasting = armManualCoast.load();
	const bool manualControlFinished =
	    requestedPower == 0 && !requestedCoast && (previousPower != 0 || previouslyCoasting);
	if (manualControlFinished) {
		const std::int32_t currentPosition = armRotationSensor.get_position();
		if (currentPosition != PROS_ERR) {
			setArmTarget(currentPosition / 100.0);
		}
	}

	if (requestedCoast && !previouslyCoasting) {
		armManualCoast.store(true);
		armMotor.set_brake_mode(pros::MotorBrake::coast);
	} else if (!requestedCoast && previouslyCoasting) {
		armMotor.set_brake_mode(armHoldingEnabled ? pros::MotorBrake::hold
		                                          : pros::MotorBrake::coast);
		armManualCoast.store(false);
	}

	armManualPower.store(requestedPower);
}

// L1 manually raises the cascade and D-pad up lowers it. Releasing the button
// captures the encoder position so the automatic controller holds it there.
void handleManualCascadeControl() {
	const bool upHeld = controller.get_digital(cascadeManualUpButton);
	const bool downHeld = controller.get_digital(cascadeManualDownButton);
	std::int32_t requestedPower = 0;

	// If both buttons are held, stop instead of choosing an unexpected direction.
	if (upHeld != downHeld) {
		requestedPower = upHeld ? cascadeManualSpeed : -cascadeManualSpeed;
	}

	const std::int32_t previousPower = cascadeManualPower.load();
	if (requestedPower == 0 && previousPower != 0) {
		const double currentPosition = cascadeMotor.get_position();
		if (std::isfinite(currentPosition)) {
			setCascadeTarget(currentPosition);
		}
	}

	cascadeManualPower.store(requestedPower);
}

// While R1 is held, lower only the cascade. On release, this is where the claw
// will open once it has been added, then both mechanisms return to position 1.
void handleCascadeDropAndReturn() {
	static bool wasHeld = false;
	const bool isHeld = controller.get_digital(cascadeDropButton);

	if (isHeld) {
		setCascadeTarget(0.0);
	} else if (wasHeld) {
		OpenClaw.extend(); // Extended is the open-claw state.
		pros::delay(1000); // Wait 250 ms for the claw to open.
		goToPositionOne();
	}

	wasHeld = isHeld;
}

// Where the rest of the code went:
//   22020Vlib/ramsete.{hpp,cpp}  trajectory following and path loading
//   22020Vlib/dsr.{hpp,cpp}      distance sensor odometry reset
//   22020Vlib/drive.{hpp,cpp}    simple_drive_distance / turn / until_wall
//   src/autons/                  one file per routine, listed in autons/autons.hpp
//   robot/config.hpp             every tunable number, including drivetrain geometry

//robot functions driver and auton end

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}


/**
 * THE AUTON YOU WANT TO RUN GOES IN THE AUTONOMOUS FUNCTION AT THE BOTTOM
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
	// Planner-exported testing curve Ramsete test.
	ramsete_auton_example();
	// To run the five planner exports in static/ instead, call:
	//   bot1_auton1();
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
    // controller
    // loop to continuously update motors

    while (true) {
		displayMechanismDegrees();

		// One new-press event equals one preset change; holding L1 does not repeat.
		if (controller.get_digital_new_press(nextPositionButton)) {
			togglePosition();
		}
		if (controller.get_digital_new_press(positionOneButton)) {
			goToPositionOne();
		}
		if (controller.get_digital_new_press(openClawToggleButton)) {
			toggleOpenClaw();
		}
		if (controller.get_digital_new_press(variableClawToggleButton)) {
			toggleVariableClaw();
		}
		handleOuttakeControl();
		handleManualArmControl();
		handleManualCascadeControl();
		handleCascadeDropAndReturn();

		// get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        //chassis.tank(leftY, rightY);
		// A-button drivetrain test disabled.
		// if (controller.get_digital_new_press(DIGITAL_A)) {
		// 	chassis.setPose(0, 0, 0);
		// 	chassis.tank(-115, 125);
		// 	pros::delay(400);
		// 	chassis.turnToHeading(345, 300, {.minSpeed = 110});
		// 	std::cout << chassis.getPose().theta;
		// }
        
        chassis.arcade(leftY, rightX);
        // delay to save resources
        pros::delay(50);
    }
}
