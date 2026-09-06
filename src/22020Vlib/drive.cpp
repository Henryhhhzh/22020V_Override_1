#include "22020Vlib/drive.hpp"

#include "robot/config.hpp"
#include "robot/hardware.hpp"
#include "pros/rtos.hpp"
#include <cmath>
#include <cstdlib>

// Basic straight movement measured with the integrated encoders on the first
// blue motor in each group. This does not use odometry, DSR, or LemLib motion.
void simple_drive_distance(double distanceInches, std::int32_t power) {
	constexpr double pi = 3.141592653589793;
	constexpr double motorRevolutionsPerWheelRevolution =
	    kMotorCartridgeRpm / kDriveWheelRpm;
	constexpr std::uint32_t safetyTimeoutMs = 3000;

	const double targetMotorDegrees =
	    std::abs(distanceInches) * 360.0 * motorRevolutionsPerWheelRevolution /
	    (pi * kDriveWheelDiameterIn);
	const std::int32_t direction = distanceInches < 0.0 ? -1 : 1;
	const std::int32_t driveCommand = direction * std::abs(power);

	leftMotors.set_encoder_units(pros::MotorUnits::degrees, 0);
	rightMotors.set_encoder_units(pros::MotorUnits::degrees, 0);
	leftMotors.tare_position(0);
	rightMotors.tare_position(0);
	leftMotors.set_brake_mode_all(pros::MotorBrake::brake);
	rightMotors.set_brake_mode_all(pros::MotorBrake::brake);

	leftMotors.move(driveCommand);
	rightMotors.move(driveCommand);
	const std::uint32_t movementStart = pros::millis();

	while (pros::millis() - movementStart < safetyTimeoutMs) {
		const double leftDegrees = std::abs(leftMotors.get_position(0));
		const double rightDegrees = std::abs(rightMotors.get_position(0));
		if (!std::isfinite(leftDegrees) || !std::isfinite(rightDegrees)) break;

		const double averageMotorDegrees = (leftDegrees + rightDegrees) / 2.0;
		if (averageMotorDegrees >= targetMotorDegrees) break;
		pros::delay(10);
	}

	leftMotors.brake();
	rightMotors.brake();
	pros::delay(200);
}

// Relative encoder-only point turn. Positive degrees turn right; negative
// degrees turn left. This does not read or target an IMU heading.
void simple_turn_degrees(double turnDegrees, std::int32_t power) {
	constexpr double motorRevolutionsPerWheelRevolution =
	    kMotorCartridgeRpm / kDriveWheelRpm;
	constexpr double turnCalibration = 1.0;
	constexpr std::uint32_t safetyTimeoutMs = 3000;

	if (std::abs(turnDegrees) < 0.01 || power == 0) return;

	// During a point turn, each wheel travels an arc around the robot's center.
	const double targetMotorDegrees =
	    std::abs(turnDegrees) * kTrackWidthIn / kDriveWheelDiameterIn *
	    motorRevolutionsPerWheelRevolution * turnCalibration;
	const std::int32_t direction = turnDegrees > 0.0 ? 1 : -1;
	const std::int32_t turnPower = std::abs(power);

	leftMotors.set_encoder_units(pros::MotorUnits::degrees, 0);
	rightMotors.set_encoder_units(pros::MotorUnits::degrees, 0);
	leftMotors.tare_position(0);
	rightMotors.tare_position(0);
	leftMotors.set_brake_mode_all(pros::MotorBrake::brake);
	rightMotors.set_brake_mode_all(pros::MotorBrake::brake);

	leftMotors.move(direction * turnPower);
	rightMotors.move(-direction * turnPower);
	const std::uint32_t movementStart = pros::millis();

	while (pros::millis() - movementStart < safetyTimeoutMs) {
		const double leftDegrees = std::abs(leftMotors.get_position(0));
		const double rightDegrees = std::abs(rightMotors.get_position(0));
		if (!std::isfinite(leftDegrees) || !std::isfinite(rightDegrees)) break;

		const double averageMotorDegrees = (leftDegrees + rightDegrees) / 2.0;
		if (averageMotorDegrees >= targetMotorDegrees) break;
		pros::delay(10);
	}

	leftMotors.brake();
	rightMotors.brake();
	pros::delay(200);
}

// Drive backward until the drivetrain is loaded and nearly stopped, as when
// contacting a wall. Motor voltage is commanded by us, so current plus velocity
// are the useful stall signals. The timeout prevents indefinite pushing.
void simple_drive_backward_until_wall(std::int32_t power, std::uint32_t timeoutMs) {
	constexpr std::uint32_t startupIgnoreMs = 350;
	constexpr std::uint32_t wallContactConfirmMs = 250;
	constexpr double wallContactVelocityRpm = 8.0;
	constexpr std::int32_t wallContactCurrentMilliamps = 800;

	std::int32_t drivePower = std::abs(power);
	if (drivePower == 0 || timeoutMs == 0) return;
	if (drivePower > 127) drivePower = 127;

	leftMotors.set_brake_mode_all(pros::MotorBrake::brake);
	rightMotors.set_brake_mode_all(pros::MotorBrake::brake);
	leftMotors.move(-drivePower);
	rightMotors.move(-drivePower);

	const std::uint32_t movementStart = pros::millis();
	std::uint32_t wallContactStart = 0;

	while (pros::millis() - movementStart < timeoutMs) {
		const std::uint32_t elapsedMs = pros::millis() - movementStart;
		const double leftVelocity = std::abs(leftMotors.get_actual_velocity(0));
		const double rightVelocity = std::abs(rightMotors.get_actual_velocity(0));
		const std::int32_t leftCurrent = leftMotors.get_current_draw(0);
		const std::int32_t rightCurrent = rightMotors.get_current_draw(0);

		if (!std::isfinite(leftVelocity) || !std::isfinite(rightVelocity) ||
		    leftCurrent < 0 || rightCurrent < 0) {
			break;
		}

		const double averageVelocity = (leftVelocity + rightVelocity) / 2.0;
		const std::int32_t averageCurrent = (leftCurrent + rightCurrent) / 2;
		const bool wallContact = elapsedMs >= startupIgnoreMs &&
		                         averageVelocity <= wallContactVelocityRpm &&
		                         averageCurrent >= wallContactCurrentMilliamps;

		if (wallContact) {
			if (wallContactStart == 0) wallContactStart = pros::millis();
			if (pros::millis() - wallContactStart >= wallContactConfirmMs) break;
		} else {
			wallContactStart = 0;
		}

		pros::delay(10);
	}

	leftMotors.brake();
	rightMotors.brake();
}
