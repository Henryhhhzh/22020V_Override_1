#include "22020Vlib/drive.hpp"

#include "robot/config.hpp"
#include "robot/hardware.hpp"
#include "pros/error.h"
#include "pros/rtos.hpp"
#include <cmath>
#include <cstdlib>

namespace {
// Read in degrees without changing the motor's units or zero. LemLib uses the
// same encoders for odometry, so changing either would corrupt its position.
bool motorDegrees(const pros::MotorGroup& motors, double& degrees) {
	const double position = motors.get_position(0);
	if (!std::isfinite(position)) return false;
	switch (motors.get_encoder_units(0)) {
		case pros::MotorUnits::degrees:
			degrees = position;
			break;
		case pros::MotorUnits::rotations:
			degrees = position * 360.0;
			break;
		case pros::MotorUnits::counts: {
			// V5 motor encoder counts per output-shaft revolution by cartridge.
			double countsPerRevolution;
			switch (motors.get_gearing(0)) {
				case pros::MotorGears::red: countsPerRevolution = 1800.0; break;
				case pros::MotorGears::green: countsPerRevolution = 900.0; break;
				case pros::MotorGears::blue: countsPerRevolution = 300.0; break;
				default: return false;
			}
			degrees = position * 360.0 / countsPerRevolution;
			break;
		}
		default: return false;
	}
	return std::isfinite(degrees);
}

bool stopDrive(bool completed, std::uint32_t settleMs = 0) {
	leftMotors.brake();
	rightMotors.brake();
	if (settleMs > 0) pros::delay(settleMs);
	return completed && !pros::competition::is_disabled();
}

bool validPower(std::int32_t power) {
	return power != 0 && power >= -127 && power <= 127;
}

// Signed progress on BOTH sides is required. A backwards or stationary side
// must not be hidden by the other side spinning farther than requested.
bool encoderMove(double targetDegrees, int leftDirection, int rightDirection,
                 std::int32_t power) {
	constexpr std::uint32_t safetyTimeoutMs = 3000;
	double leftStart;
	double rightStart;
	if (pros::competition::is_disabled() || !std::isfinite(targetDegrees) ||
	    !motorDegrees(leftMotors, leftStart) || !motorDegrees(rightMotors, rightStart)) {
		return stopDrive(false);
	}
	if (targetDegrees <= 0.0) return stopDrive(true, 200);
	if (leftMotors.set_brake_mode_all(pros::MotorBrake::brake) == PROS_ERR ||
	    rightMotors.set_brake_mode_all(pros::MotorBrake::brake) == PROS_ERR ||
	    leftMotors.move(leftDirection * std::abs(power)) == PROS_ERR ||
	    rightMotors.move(rightDirection * std::abs(power)) == PROS_ERR) {
		return stopDrive(false);
	}
	const std::uint32_t movementStart = pros::millis();
	bool completed = false;
	while (pros::millis() - movementStart < safetyTimeoutMs) {
		if (pros::competition::is_disabled()) break;
		double leftDegrees;
		double rightDegrees;
		if (!motorDegrees(leftMotors, leftDegrees) ||
		    !motorDegrees(rightMotors, rightDegrees)) break;
		if ((leftDegrees - leftStart) * leftDirection >= targetDegrees &&
		    (rightDegrees - rightStart) * rightDirection >= targetDegrees) {
			completed = true;
			break;
		}
		pros::delay(10);
	}
	return stopDrive(completed, 200);
}
} // namespace

// Encoder-only movement, using a local reference instead of resetting odometry.
bool simple_drive_distance(double distanceInches, std::int32_t power) {
	if (!std::isfinite(distanceInches) || !validPower(power)) return stopDrive(false);
	constexpr double pi = 3.141592653589793;
	constexpr double motorRevolutionsPerWheelRevolution =
	    kMotorCartridgeRpm / kDriveWheelRpm;
	const double targetMotorDegrees =
	    std::abs(distanceInches) * 360.0 * motorRevolutionsPerWheelRevolution /
	    (pi * kDriveWheelDiameterIn);
	const int direction = distanceInches < 0.0 ? -1 : 1;
	return encoderMove(targetMotorDegrees, direction, direction, power);
}

// Relative encoder-only point turn. Positive degrees turn right; negative
// degrees turn left. This does not read or target an IMU heading.
bool simple_turn_degrees(double turnDegrees, std::int32_t power) {
	if (!std::isfinite(turnDegrees) || !validPower(power)) return stopDrive(false);
	if (std::abs(turnDegrees) < 0.01) return stopDrive(true);
	constexpr double motorRevolutionsPerWheelRevolution =
	    kMotorCartridgeRpm / kDriveWheelRpm;
	constexpr double turnCalibration = 1.0;
	const double targetMotorDegrees =
	    std::abs(turnDegrees) * kTrackWidthIn / kDriveWheelDiameterIn *
	    motorRevolutionsPerWheelRevolution * turnCalibration;
	const int direction = turnDegrees > 0.0 ? 1 : -1;
	return encoderMove(targetMotorDegrees, direction, -direction, power);
}

// Drive backward until the drivetrain is loaded and nearly stopped, as when
// contacting a wall. Return false if contact cannot be confirmed in time.
bool simple_drive_backward_until_wall(std::int32_t power, std::uint32_t timeoutMs) {
	constexpr std::uint32_t startupIgnoreMs = 350;
	constexpr std::uint32_t wallContactConfirmMs = 250;
	constexpr double wallContactVelocityRpm = 8.0;
	constexpr std::int32_t wallContactCurrentMilliamps = 800;
	if (!validPower(power) || timeoutMs == 0 || pros::competition::is_disabled()) {
		return stopDrive(false);
	}
	const std::int32_t drivePower = std::abs(power);
	if (leftMotors.set_brake_mode_all(pros::MotorBrake::brake) == PROS_ERR ||
	    rightMotors.set_brake_mode_all(pros::MotorBrake::brake) == PROS_ERR ||
	    leftMotors.move(-drivePower) == PROS_ERR ||
	    rightMotors.move(-drivePower) == PROS_ERR) {
		return stopDrive(false);
	}
	const std::uint32_t movementStart = pros::millis();
	std::uint32_t wallContactStart = 0;
	bool confirmingContact = false;
	bool completed = false;
	while (pros::millis() - movementStart < timeoutMs) {
		if (pros::competition::is_disabled()) break;
		const std::uint32_t elapsedMs = pros::millis() - movementStart;
		const double leftVelocity = std::abs(leftMotors.get_actual_velocity(0));
		const double rightVelocity = std::abs(rightMotors.get_actual_velocity(0));
		const std::int32_t leftCurrent = leftMotors.get_current_draw(0);
		const std::int32_t rightCurrent = rightMotors.get_current_draw(0);
		if (!std::isfinite(leftVelocity) || !std::isfinite(rightVelocity) ||
		    leftCurrent == PROS_ERR || rightCurrent == PROS_ERR ||
		    leftCurrent < 0 || rightCurrent < 0) break;
		const double averageVelocity = (leftVelocity + rightVelocity) / 2.0;
		// Convert before adding to avoid overflowing the integer telemetry type.
		const double averageCurrent = (static_cast<double>(leftCurrent) + rightCurrent) / 2.0;
		const bool wallContact = elapsedMs >= startupIgnoreMs &&
		                         averageVelocity <= wallContactVelocityRpm &&
		                         averageCurrent >= wallContactCurrentMilliamps;
		if (wallContact) {
			if (!confirmingContact) {
				wallContactStart = pros::millis();
				confirmingContact = true;
			}
			if (pros::millis() - wallContactStart >= wallContactConfirmMs) {
				completed = true;
				break;
			}
		} else {
			confirmingContact = false;
		}
		pros::delay(10);
	}
	return stopDrive(completed);
}
