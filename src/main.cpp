#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include <atomic>
#include <cmath>

pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup leftMotors({1, -2, -3}, pros::MotorGearset::blue); // left drive motors, reversed
pros::MotorGroup rightMotors({-10, 9, 8}, pros::MotorGearset::blue); // right drive motors

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

// individual mechanism motors
pros::Motor armMotor(4); // normal direction; arm sensor remains reversed
pros::Motor cascadeMotor(-20);
pros::Motor intakeMotor(11);

// Double-acting pneumatic valves on the V5 brain's three-wire ADI ports.
// Both start retracted; extend() and retract() switch their two positions.
pros::adi::Pneumatics VariableClaw('H', false);
pros::adi::Pneumatics OpenClaw('F', false, true); // Inverted: retracted is physically closed.

// Auxiliary Rotation Sensor on port 5; not used for odometry.
pros::Rotation armRotationSensor(-5);

// The mechanism controller owns both motors continuously. Targets are stored
// in centidegrees so changing a target is atomic on the V5 brain.
std::atomic<std::int32_t> armTargetCentidegrees{0};
std::atomic<std::int32_t> cascadeTargetCentidegrees{0};
std::atomic<std::int32_t> armManualPower{0};
std::atomic<bool> armManualCoast{false};
std::atomic<std::int32_t> cascadeManualPower{0};

void mechanismPositionController();


// Inertial sensor on placeholder port 7
pros::Imu imu(7);


// tracking wheels
// Tracking-wheel sensor ports are placeholders until final wiring is known.
// Horizontal tracking wheel encoder: Rotation Sensor on port 18, reversed.
pros::Rotation horizontalEnc(-18);
// Vertical tracking wheel encoder: Rotation Sensor on port 19, reversed.
pros::Rotation verticalEnc(-19);
// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_2, -4.523);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_2, -0.5);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              10.1, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              450, // drivetrain rpm is 360
                              1 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(5.4, // proportional gain (kP)
                                            0.2, // integral gain (kI)
                                            3.3, // derivative gain (kD)
                                            3, // anti windup
                                            .8, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            2, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)   
);

// angular motion controller
lemlib::ControllerSettings angularController(2, // proportional gain (kP)
                                             0.1, // integral gain (kI)
                                             13, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            &horizontal, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
	                                  10, // minimum output where drivetrain will move out of 127
	                                  1.015 // expo curve gain
);

// DSR wall locations
constexpr float dsrTopWallY = 72.0; // top wall Y coordinate, in inches
constexpr float dsrRightWallX = 72.0; // right wall X coordinate, in inches
constexpr float dsrBottomWallY = -72.0; // bottom wall Y coordinate, in inches
constexpr float dsrLeftWallX = -72.0; // left wall X coordinate, in inches

// DSR reading limits
constexpr float dsrMinValidDistance = 0.5; // minimum valid distance sensor reading, in inches
constexpr float dsrMaxValidDistance = 100.0; // maximum valid distance sensor reading, in inches

// DSR sensor offsets are measured from the tracking point to the distance sensor lens.
// These are sensor-local offsets, so they are different for each physical sensor:
// x_offset = forward/back along the direction that sensor points. Positive is outward.
// y_offset = side offset to that sensor's right. Negative is to that sensor's left.
// Example: front sensor 5" in front of tracking point and 1" to robot right -> (0, 5, 1).
// Example: right sensor 4" to robot right and 0.5" toward robot front -> (90, 4, -0.5).
// These offset coordinates are sensor-local, not field-global.
// DSR front physical sensor settings
DsrSensorConfig dsrFrontSensor(0, // heading offset from robot front, in degrees
	                               0, // x_offset: positive toward robot front, in inches
	                               0 // y_offset: positive toward robot right, in inches
);

// DSR right physical sensor settings
DsrSensorConfig dsrRightSensor(90, // heading offset from robot front, in degrees
	                               0, // x_offset: positive toward robot right, in inches
	                               0 // y_offset: positive toward robot back, in inches
);

// DSR back physical sensor settings
DsrSensorConfig dsrBackSensor(180, // heading offset from robot front, in degrees
	                              0, // x_offset: positive toward robot back, in inches
	                              0 // y_offset: positive toward robot left, in inches
);

// DSR left physical sensor settings
DsrSensorConfig dsrLeftSensor(270, // heading offset from robot front, in degrees
	                              0, // x_offset: positive toward robot left, in inches
	                              0 // y_offset: positive toward robot front, in inches
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

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

	// Mechanism positions use degrees and are absolute from their startup zero.
	// Targets are not wrapped, so values such as 400 degrees remain valid.
	cascadeMotor.set_encoder_units(pros::MotorUnits::degrees);
	cascadeMotor.tare_position();
	armRotationSensor.reset_position();
	cascadeMotor.set_brake_mode(pros::MotorBrake::hold);
	armMotor.set_brake_mode(pros::MotorBrake::hold);
	// The main claw's retracted state is its closed starting state.
	OpenClaw.retract();
	armTargetCentidegrees.store(0);
	cascadeTargetCentidegrees.store(0);
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

constexpr double mechanismTargetToleranceDegrees = 25.0;
constexpr std::int32_t mechanismTargetToleranceCentidegrees = 2500;
constexpr std::uint32_t mechanismSettleTimeMs = 100;
constexpr std::uint32_t mechanismLoopMs = 10;
constexpr double armMinimumDegrees = 0.0;
constexpr double armMaximumDegrees = 100.0;
constexpr double cascadeMinimumDegrees = 0.0;
constexpr double cascadeMaximumDegrees = 1350.0;
constexpr std::int32_t armMinimumCentidegrees = 0;
constexpr std::int32_t armMaximumCentidegrees = 10000;

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
                       double maximumPower) {
	double normalizedError = errorDegrees / slowdownDegrees;
	normalizedError = std::fmax(-1.0, std::fmin(1.0, normalizedError));
	const double magnitude = std::abs(normalizedError);
	const double curvedMagnitude = magnitude * magnitude * (3.0 - 2.0 * magnitude);

	if (normalizedError >= 0) {
		return holdPower + (maximumPower - holdPower) * curvedMagnitude;
	}
	return holdPower - (maximumPower + holdPower) * curvedMagnitude;
}

// This is the only task that commands the arm and cascade motors. It never
// times out: after reaching a target, it keeps correcting both mechanisms.
void mechanismPositionController() {
	// Increase a hold power if that mechanism still rests below its target;
	// decrease it if the mechanism steadily creeps upward.
	constexpr double armUpwardHoldPower = 45.0;
	constexpr double armMaximumMovePower = 85.0;
	constexpr double cascadeUpwardHoldPower = 25.0;

	// The arm uses its curve across most of a normal move for gentler motion.
	// The cascade keeps its separate linear slowdown behavior.
	constexpr double armSlowdownDegrees = 100.0;
	constexpr double cascadeSlowdownDegrees = 150.0;
	constexpr double fullPowerKickMinimumErrorDegrees = 1.0;

	std::int32_t armPreviousTarget = 0;
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
		} else {
			const double armError = (armTarget - armPosition) / 100.0;
			const bool targetChanged = !armPidInitialized || armTarget != armPreviousTarget;
			if (targetChanged) {
				armPreviousTarget = armTarget;
				armPidInitialized = true;
			}

			// Position zero receives no upward feedforward, so position 1 can rest at
			// the bottom. Every raised target gets gravity compensation.
			const double armHoldPower = armTarget > 0 ? armUpwardHoldPower : 0.0;
			double armOutput =
			    curvedArmOutput(armError, armSlowdownDegrees, armHoldPower,
			                    armMaximumMovePower);
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
// There is no overall timeout; the background controller keeps holding it even
// after this function returns. A newer target cancels this wait safely.
void cascadeSpinToDegree(double degrees) {
	setCascadeTarget(degrees);
	const std::int32_t requestedTarget = cascadeTargetCentidegrees.load();
	const double requestedDegrees = requestedTarget / 100.0;
	std::uint32_t settledSince = 0;

	while (cascadeTargetCentidegrees.load() == requestedTarget) {
		const double position = cascadeMotor.get_position();
		if (!std::isfinite(position)) return;

		const double error = requestedDegrees - position;
		if (std::abs(error) <= mechanismTargetToleranceDegrees) {
			if (settledSince == 0) settledSince = pros::millis();
			if (pros::millis() - settledSince >= mechanismSettleTimeMs) return;
		} else {
			settledSince = 0;
		}
		pros::delay(mechanismLoopMs);
	}
}

// Set an absolute arm target and wait until the port-5 Rotation Sensor remains
// within 25 degrees. The persistent controller continues holding afterward.
void armSpinToDegree(double degrees) {
	setArmTarget(degrees);
	const std::int32_t requestedTarget = armTargetCentidegrees.load();
	std::uint32_t settledSince = 0;

	while (armTargetCentidegrees.load() == requestedTarget) {
		const std::int32_t position = armRotationSensor.get_position();
		if (position == PROS_ERR) return;

		if (std::abs(requestedTarget - position) <= mechanismTargetToleranceCentidegrees) {
			if (settledSince == 0) settledSince = pros::millis();
			if (pros::millis() - settledSince >= mechanismSettleTimeMs) return;
		} else {
			settledSince = 0;
		}
		pros::delay(mechanismLoopMs);
	}
}

// Each numbered position combines an arm target and a cascade target.
struct PositionTargets {
	double armDegrees;
	double cascadeDegrees;
};

// Array index 0 is position 1; array index 5 is position 6.
constexpr PositionTargets positions[6] = {
	{0.0, 0.0},    // position 1  // position 2
	{10.0, 1300.0}, // position 3
	{95.0, 0.0},   // position 4
	{100.0, 950.0},// position 5
	{105.0, 1300.0},// position 6
	{105.0, 1300.0}

};

constexpr auto nextPositionButton = pros::E_CONTROLLER_DIGITAL_Y;
constexpr auto positionOneButton = pros::E_CONTROLLER_DIGITAL_R1;
constexpr auto armManualUpButton = pros::E_CONTROLLER_DIGITAL_LEFT;
constexpr auto armManualDownButton = pros::E_CONTROLLER_DIGITAL_UP;
constexpr std::int32_t armManualSpeed = 75;
constexpr auto cascadeDropButton = pros::E_CONTROLLER_DIGITAL_RIGHT;
constexpr auto openClawToggleButton = pros::E_CONTROLLER_DIGITAL_R2;
constexpr auto variableClawToggleButton = pros::E_CONTROLLER_DIGITAL_Y;
constexpr auto cascadeManualUpButton = pros::E_CONTROLLER_DIGITAL_L1;
constexpr auto cascadeManualDownButton = pros::E_CONTROLLER_DIGITAL_L2;
constexpr std::int32_t cascadeManualSpeed = 127;
constexpr auto intakeButton = pros::E_CONTROLLER_DIGITAL_RIGHT;
constexpr std::int32_t intakeSpeed = -127;
constexpr double clawCloseCascadeLiftDegrees = 150.0;

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

// Run the intake only while the controller's right D-pad button is held.
void handleIntakeControl() {
	intakeMotor.move(controller.get_digital(intakeButton) ? intakeSpeed : 0);
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
		armMotor.set_brake_mode(pros::MotorBrake::hold);
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

//DISTANCE SENSOR RESET FUNCTIONS

float dsr_deg_to_rad(float degrees) {
	return degrees * 3.1415926535 / 180.0;
}

float dsr_normalize_heading(float heading) {
	while (heading < 0) heading += 360;
	while (heading >= 360) heading -= 360;
	return heading;
}

DsrSensorConfig dsr_get_sensor_config(RobotDsrSensor sensor) {
	switch (sensor) {
		case RobotDsrSensor::front:
			return dsrFrontSensor;
		case RobotDsrSensor::right:
			return dsrRightSensor;
		case RobotDsrSensor::back:
			return dsrBackSensor;
		case RobotDsrSensor::left:
			return dsrLeftSensor;
	}

	return {0, 0, 0};
}

float dsr_read_sensor_inches(RobotDsrSensor sensor) {
	// TODO: return the selected physical distance sensor's reading in inches.
	(void)sensor;
	return -1;
}

float dsr_wall_angle(DsrWall wall) {
	switch (wall) {
		case DsrWall::top:
			return 0;
		case DsrWall::right:
			return 90;
		case DsrWall::bottom:
			return 180;
		case DsrWall::left:
			return 270;
	}

	return 0;
}

DsrWall dsr_nearest_wall_for_sensor(RobotDsrSensor sensor, float heading) {
	DsrSensorConfig config = dsr_get_sensor_config(sensor);
	float sensor_heading = dsr_normalize_heading(heading + config.heading_offset);

	if (sensor_heading >= 315 || sensor_heading < 45) {
		return DsrWall::top;
	} else if (sensor_heading < 135) {
		return DsrWall::right;
	} else if (sensor_heading < 225) {
		return DsrWall::bottom;
	}

	return DsrWall::left;
}

float dsr_tracking_point_distance_to_wall(RobotDsrSensor sensor, DsrWall wall, float heading) {
	float sensor_distance = dsr_read_sensor_inches(sensor);
	if (sensor_distance < dsrMinValidDistance || sensor_distance > dsrMaxValidDistance) {
		return -1;
	}

	DsrSensorConfig config = dsr_get_sensor_config(sensor);
	float theta = dsr_deg_to_rad((heading + config.heading_offset) - dsr_wall_angle(wall));
	float blue = std::cos(theta) * sensor_distance;
	float purple = std::cos(theta) * config.x_offset;
	float green = std::sin(theta) * config.y_offset * -1;

	return blue + purple + green;
}

void dsr_add_coordinate_sample(float& x_sum, int& x_count, float& y_sum, int& y_count,
                               RobotDsrSensor sensor, DsrWall wall, float heading) {
	float distance_to_wall = dsr_tracking_point_distance_to_wall(sensor, wall, heading);
	if (distance_to_wall < 0) {
		return;
	}

	switch (wall) {
		case DsrWall::top:
			y_sum += dsrTopWallY - distance_to_wall;
			y_count++;
			break;
		case DsrWall::bottom:
			y_sum += dsrBottomWallY + distance_to_wall;
			y_count++;
			break;
		case DsrWall::right:
			x_sum += dsrRightWallX - distance_to_wall;
			x_count++;
			break;
		case DsrWall::left:
			x_sum += dsrLeftWallX + distance_to_wall;
			x_count++;
			break;
	}
}

void dsr_add_sensor_coordinate_sample(float& x_sum, int& x_count, float& y_sum, int& y_count,
                                      RobotDsrSensor sensor, float heading) {
	DsrWall wall = dsr_nearest_wall_for_sensor(sensor, heading);
	dsr_add_coordinate_sample(x_sum, x_count, y_sum, y_count, sensor, wall, heading);
}

/**
 * Runs during auto to reset odom with any combination of physical distance sensors.
 * Argument order: front, right, back, left.
 */
void dsr(bool use_front_sensor, bool use_right_sensor, bool use_back_sensor, bool use_left_sensor) {
	lemlib::Pose pose = chassis.getPose();
	float heading = dsr_normalize_heading(pose.theta);

	float x_sum = 0;
	float y_sum = 0;
	int x_count = 0;
	int y_count = 0;

	if (use_front_sensor) {
		dsr_add_sensor_coordinate_sample(x_sum, x_count, y_sum, y_count, RobotDsrSensor::front, heading);
	}
	if (use_right_sensor) {
		dsr_add_sensor_coordinate_sample(x_sum, x_count, y_sum, y_count, RobotDsrSensor::right, heading);
	}
	if (use_back_sensor) {
		dsr_add_sensor_coordinate_sample(x_sum, x_count, y_sum, y_count, RobotDsrSensor::back, heading);
	}
	if (use_left_sensor) {
		dsr_add_sensor_coordinate_sample(x_sum, x_count, y_sum, y_count, RobotDsrSensor::left, heading);
	}

	float new_x = x_count > 0 ? x_sum / x_count : pose.x;
	float new_y = y_count > 0 ? y_sum / y_count : pose.y;
	if (x_count > 0 || y_count > 0) {
		chassis.setPose(new_x, new_y, pose.theta);
	}
}

//RAMSETE TRAJECTORY FOLLOWER

// One row of a planner-exported trajectory.
// theta is a math-frame angle in radians: 0 = +X (field right), counter-clockwise
// positive, exactly as the planner writes theta_rad. v is inches/second, omega is
// radians/second. x and y are LemLib odom inches (x = right, y = forward).
struct RamsetePoint {
	float t; // time_s
	float x; // x_in  (field X, right)
	float y; // y_in  (field Y, forward)
	float theta; // theta_rad (math frame)
	float v; // v_ips
	float omega; // omega_radps
};

// Ramsete tuning. b adds aggressiveness, zeta adds damping. These two are
// convention-independent and safe to start at these values, then tune.
constexpr float kRamseteB = 2.0;
constexpr float kRamseteZeta = 0.7;

// Robot/drivetrain constants. These must match the real robot and the values
// used in the planner so the exported velocities are actually reachable.
constexpr float kRamsetePi = 3.14159265358979;
constexpr float kTrackWidthIn = 10.1; // matches the Drivetrain track width
constexpr float kDriveWheelDiameterIn = 3.25; // NEW_325 omni
constexpr float kDriveWheelRpm = 450.0; // wheel rpm, from the Drivetrain config
constexpr float kMotorCartridgeRpm = 600.0; // blue cartridge
constexpr int kRamseteLoopMs = 10; // 100 Hz control loop

// LemLib pose.theta is degrees, 0 = +Y (forward), clockwise positive.
// The planner's theta is a math angle, 0 = +X (right), counter-clockwise positive.
// Bridge the two so position and heading error are computed in one frame.
float ramsete_heading_to_math_rad(float heading_deg) {
	return (kRamsetePi / 2.0) - (heading_deg * kRamsetePi / 180.0);
}

// Wrap an angle to [-pi, pi] so heading error never blows up across +-pi.
float ramsete_normalize_angle(float radians) {
	while (radians > kRamsetePi) radians -= 2.0 * kRamsetePi;
	while (radians < -kRamsetePi) radians += 2.0 * kRamsetePi;
	return radians;
}

// Linearly sample the trajectory at an arbitrary time so the control loop rate
// is decoupled from the export dt. theta is interpolated through the shortest arc.
RamsetePoint ramsete_sample(const RamsetePoint* traj, int count, float time_s) {
	if (time_s <= traj[0].t) return traj[0];
	if (time_s >= traj[count - 1].t) return traj[count - 1];

	for (int i = 1; i < count; i++) {
		if (time_s <= traj[i].t) {
			const RamsetePoint& a = traj[i - 1];
			const RamsetePoint& b = traj[i];
			float span = b.t - a.t;
			float r = span <= 0.0 ? 0.0 : (time_s - a.t) / span;

			RamsetePoint out;
			out.t = time_s;
			out.x = a.x + (b.x - a.x) * r;
			out.y = a.y + (b.y - a.y) * r;
			out.theta = a.theta + ramsete_normalize_angle(b.theta - a.theta) * r;
			out.v = a.v + (b.v - a.v) * r;
			out.omega = a.omega + (b.omega - a.omega) * r;
			return out;
		}
	}

	return traj[count - 1];
}

// Convert a wheel's linear velocity (in/s) into a V5 motor velocity (cartridge
// rpm) for move_velocity, accounting for the external gear ratio, and clamp it.
float ramsete_wheel_ips_to_motor_rpm(float wheel_ips) {
	float wheel_circumference = kRamsetePi * kDriveWheelDiameterIn;
	float wheel_rpm = (wheel_ips / wheel_circumference) * 60.0;
	float motor_rpm = wheel_rpm * (kMotorCartridgeRpm / kDriveWheelRpm);

	if (motor_rpm > kMotorCartridgeRpm) motor_rpm = kMotorCartridgeRpm;
	if (motor_rpm < -kMotorCartridgeRpm) motor_rpm = -kMotorCartridgeRpm;
	return motor_rpm;
}

// Follow a planner-exported Ramsete trajectory. Blocks until the trajectory
// time elapses, then stops the drive. Seed odom to the path's first point first
// (chassis.setPose(...) or a dsr() reset) so the start pose matches the path.
void follow_ramsete(const RamsetePoint* traj, int count) {
	if (traj == nullptr || count < 2) return;

	uint32_t start_ms = pros::millis();

	while (true) {
		float t = (pros::millis() - start_ms) / 1000.0;
		if (t >= traj[count - 1].t) break;

		RamsetePoint goal = ramsete_sample(traj, count, t);
		lemlib::Pose pose = chassis.getPose(); // x, y in inches; theta in degrees
		float theta = ramsete_heading_to_math_rad(pose.theta);

		// Position and heading error in the field frame.
		float error_x = goal.x - pose.x;
		float error_y = goal.y - pose.y;
		float error_theta = ramsete_normalize_angle(goal.theta - theta);

		// Rotate the position error into the robot's local frame.
		float local_x = std::cos(theta) * error_x + std::sin(theta) * error_y;
		float local_y = -std::sin(theta) * error_x + std::cos(theta) * error_y;

		// Ramsete control law. sinc(error_theta) avoids the divide-by-zero as the
		// heading error goes to 0.
		float k = 2.0 * kRamseteZeta * std::sqrt(goal.omega * goal.omega + kRamseteB * goal.v * goal.v);
		float sinc = std::fabs(error_theta) < 1e-6 ? 1.0 : std::sin(error_theta) / error_theta;
		float v_cmd = goal.v * std::cos(error_theta) + k * local_x;
		float omega_cmd = goal.omega + k * error_theta + kRamseteB * goal.v * sinc * local_y;

		// Differential-drive split. omega is counter-clockwise positive, matching
		// the math frame, so the right wheel speeds up on a left (CCW) turn.
		float left_ips = v_cmd - (omega_cmd * kTrackWidthIn / 2.0);
		float right_ips = v_cmd + (omega_cmd * kTrackWidthIn / 2.0);

		leftMotors.move_velocity(static_cast<int>(ramsete_wheel_ips_to_motor_rpm(left_ips)));
		rightMotors.move_velocity(static_cast<int>(ramsete_wheel_ips_to_motor_rpm(right_ips)));

		pros::delay(kRamseteLoopMs);
	}

	leftMotors.move_velocity(0);
	rightMotors.move_velocity(0);
}

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
 * all different autons will be here
 * put the autons you want to run in the autonomous function at the bottom
 */

	void example_auton() {
	//auton example there should be many of these

	}

	void example_auton2() {

	}

// Example: a 24-inch forward move written in the planner's row format.
// Replace these rows with your own export. Each row is:
//   { time_s, x_in, y_in, theta_rad, v_ips, omega_radps }
static const RamsetePoint kExampleRamsetePath[] = {
	{0.0, 0.0, 0.0, 1.5708, 0.0, 0.0},
	{0.2, 0.0, 1.2, 1.5708, 12.0, 0.0},
	{0.4, 0.0, 4.8, 1.5708, 24.0, 0.0},
	{0.6, 0.0, 9.6, 1.5708, 24.0, 0.0},
	{0.8, 0.0, 14.4, 1.5708, 24.0, 0.0},
	{1.0, 0.0, 19.2, 1.5708, 24.0, 0.0},
	{1.2, 0.0, 22.8, 1.5708, 12.0, 0.0},
	{1.4, 0.0, 24.0, 1.5708, 0.0, 0.0},
};

void ramsete_auton_example() {
	int count = sizeof(kExampleRamsetePath) / sizeof(kExampleRamsetePath[0]);
	// Seed odom to the path's first point, then follow it.
	chassis.setPose(0, 0, 0);
	follow_ramsete(kExampleRamsetePath, count);
}

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
	example_auton();
    //the auton you want to run
    // To run a planner-exported Ramsete path instead, call:
    //   ramsete_auton_example();
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
		handleIntakeControl();
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
