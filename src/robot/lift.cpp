#include "robot/lift.hpp"

#include "robot/claw.hpp"
#include "robot/hardware.hpp"
#include "pros/rtos.hpp"
#include <algorithm>
#include <cmath>

namespace {
enum class LoweringPhase {
	idle, manual, waitingForPullAway, returningToBottom,
	pickupReturning, resetting
};
LoweringPhase loweringPhase = LoweringPhase::idle;
unsigned int clawOpenedAtMs = 0;
float releaseX = 0.0;
float releaseY = 0.0;
bool pullAwayConfirmed = false;
bool manualClawTogglePending = false;
enum class ResetMode { normalLift, returningToZero, atZero };
ResetMode resetMode = ResetMode::normalLift;

enum class TravelStatus { ready, limit, error };

double liftEffectiveLowerLimit() {
	// This is an absolute floor, not an offset added to the current position.
	return liftLowerLimitDegrees + (OpenClaw.is_extended() ? 0.0 : liftClawPickupDegrees);
}

bool liftAtResetPosition() {
	const auto positions = liftMotors.get_position_all();
	if (positions.empty()) return false;
	for (const double position : positions) {
		if (!std::isfinite(position) ||
		    std::fabs(position - liftLowerLimitDegrees) > liftMoveToleranceDegrees) {
			return false;
		}
	}
	return true;
}

void cancelResetTilt() {
	resetMode = ResetMode::normalLift;
	restoreClawTilt();
}

// Button resets and macro returns share the same zero and tilt-toggle state.
bool moveToResetPosition() {
	restoreClawTilt();
	if (liftAtResetPosition()) {
		resetMode = ResetMode::atZero;
		liftMotors.brake();
		return false;
	}
	const auto positions = liftMotors.get_position_all();
	if (positions.empty()) {
		cancelResetTilt();
		liftMotors.brake();
		return false;
	}
	for (const double position : positions) {
		if (!std::isfinite(position)) {
			cancelResetTilt();
			liftMotors.brake();
			return false;
		}
	}
	resetMode = ResetMode::returningToZero;
	liftMotors.move_absolute(liftLowerLimitDegrees, liftUpVelocityRpm);
	return true;
}

TravelStatus liftTravelStatus(bool raising) {
	const auto positions = liftMotors.get_position_all();
	if (positions.empty()) return TravelStatus::error;
	bool atLimit = false;
	for (const double position : positions) {
		if (!std::isfinite(position)) return TravelStatus::error;
		// Check each motor; averaging can hide a motor crossing a travel limit.
		atLimit = atLimit || (raising ? position >= liftUpperLimitDegrees
		                              : position <= liftEffectiveLowerLimit());
	}
	return atLimit ? TravelStatus::limit : TravelStatus::ready;
}

bool liftWithinPickupBand() {
	const auto positions = liftMotors.get_position_all();
	if (positions.empty()) return false;
	for (const double position : positions) {
		if (!std::isfinite(position) ||
		    std::fabs(position - liftLowerLimitDegrees) >
		        liftClawReturnMaxDegrees - liftLowerLimitDegrees) return false;
	}
	return true;
}

bool raiseToClosedFloorIfNeeded() {
	// A completed reset holds zero until another lift command leaves reset mode.
	if (resetMode != ResetMode::normalLift) return false;
	if (OpenClaw.is_extended()) return false;
	const auto positions = liftMotors.get_position_all();
	if (positions.empty()) return false;
	bool belowFloor = false;
	for (const double position : positions) {
		if (!std::isfinite(position)) return false;
		belowFloor = belowFloor || position < liftEffectiveLowerLimit();
	}
	if (!belowFloor) return false;
	// Leave scoring HOLD positions above the floor completely alone.
	restoreClawTilt();
	liftMotors.move_absolute(liftEffectiveLowerLimit(), liftUpVelocityRpm);
	return true;
}

// Returns true while a movement toward the current floor is still required.
bool moveToLiftMinimum(int velocityRpm) {
	const TravelStatus travel = liftTravelStatus(false);
	if (travel == TravelStatus::error) {
		restoreClawTilt();
		liftMotors.brake();
		return false;
	}
	if (raiseToClosedFloorIfNeeded()) return true;
	if (travel == TravelStatus::limit) {
		liftMotors.brake();
		return false;
	}
	restoreClawTilt();
	liftMotors.move_absolute(liftEffectiveLowerLimit(), velocityRpm);
	return true;
}
} // namespace

void requestLiftPickupAssist(bool clawClosed) {
	if (pros::competition::is_disabled()) return;
	cancelResetTilt();
	// The manual toggle must win if D-pad Down is also released in this driver tick.
	// lift_down() consumes this even when another lift control has priority.
	manualClawTogglePending = true;
	// Keep manual claw toggles independent during the scoring wait/return.
	const bool pickupOrIdle = loweringPhase == LoweringPhase::idle ||
	                         loweringPhase == LoweringPhase::pickupReturning;
	if (!pickupOrIdle ||
	    controller.get_digital(liftUpButton) || controller.get_digital(liftDownButton) ||
	    controller.get_digital(liftManualDownButton) ||
	    controller.get_digital(liftResetButton)) return;
	if (!clawClosed && liftWithinPickupBand()) {
		// An OPEN click near the floor requests zero; an OPEN click high up holds.
		loweringPhase = LoweringPhase::pickupReturning;
	} else {
		// Closing above the raised floor must not pull the lift down to it.
		// The driver loop independently enforces clearance if it is below it.
		loweringPhase = LoweringPhase::idle;
	}
}

void lift_reset() {
	if (pros::competition::is_disabled()) {
		cancelResetTilt();
		loweringPhase = LoweringPhase::idle;
		manualClawTogglePending = false;
		liftMotors.brake();
		return;
	}
	const bool resetHeld = controller.get_digital(liftResetButton);
	const bool otherLiftHeld = controller.get_digital(liftUpButton) ||
	                           controller.get_digital(liftDownButton) ||
	                           controller.get_digital(liftManualDownButton);
	if (otherLiftHeld) cancelResetTilt();
	// Recognize completion before processing this click, including when the
	// encoders first reach zero in the same iteration as a fresh A press.
	if (resetMode == ResetMode::returningToZero &&
	    (loweringPhase == LoweringPhase::resetting ||
	     loweringPhase == LoweringPhase::returningToBottom ||
	     loweringPhase == LoweringPhase::pickupReturning) && liftAtResetPosition()) {
		resetMode = ResetMode::atZero;
		loweringPhase = LoweringPhase::idle;
		liftMotors.brake();
	}
	if (resetMode == ResetMode::atZero && !liftAtResetPosition()) cancelResetTilt();
	if (controller.get_digital_new_press(liftResetButton)) {
		if (resetMode == ResetMode::atZero && !otherLiftHeld &&
		    loweringPhase == LoweringPhase::idle && liftAtResetPosition()) {
			if (ClawTilt.is_extended()) tiltClawForIntake();
			else restoreClawTilt();
			liftMotors.brake();
			return;
		}
		restoreClawTilt();
		resetMode = ResetMode::returningToZero;
		loweringPhase = LoweringPhase::resetting;
	}
	if (loweringPhase != LoweringPhase::resetting) return;

	// A deliberate lift input can interrupt the reset once A is released.
	if (!resetHeld && (controller.get_digital(liftUpButton) ||
	                   controller.get_digital(liftDownButton) ||
	                   controller.get_digital(liftManualDownButton))) {
		loweringPhase = LoweringPhase::idle;
		liftMotors.brake();
		return;
	}
	if (!moveToResetPosition()) {
		loweringPhase = LoweringPhase::idle;
	}
}

void lift_up() {
	if (pros::competition::is_disabled()) {
		loweringPhase = LoweringPhase::idle;
		manualClawTogglePending = false;
		liftMotors.brake();
		return;
	}
	// lift_down() supplies the idle/release brake so it can also run the return.
	if (!controller.get_digital(liftUpButton) || controller.get_digital(liftResetButton) ||
	    loweringPhase == LoweringPhase::resetting) return;

	// Up takes priority when both buttons are held.
	cancelResetTilt();
	loweringPhase = LoweringPhase::idle;
	if (liftTravelStatus(true) != TravelStatus::ready) {
		liftMotors.brake();
		return;
	}

	liftMotors.move_absolute(liftUpperLimitDegrees, liftUpVelocityRpm);
}

void lift_manual_down() {
	if (pros::competition::is_disabled()) {
		loweringPhase = LoweringPhase::idle;
		manualClawTogglePending = false;
		liftMotors.brake();
		return;
	}
	if (!controller.get_digital(liftManualDownButton) ||
	    controller.get_digital(liftResetButton) ||
	    controller.get_digital(liftUpButton) ||
	    loweringPhase == LoweringPhase::resetting) return;

	// No release sequence: the normal idle branch in lift_down() brakes on release.
	cancelResetTilt();
	loweringPhase = LoweringPhase::idle;
	moveToLiftMinimum(liftManualDownVelocityRpm);
}

void lift_down() {
	const bool manualClawToggled = manualClawTogglePending;
	manualClawTogglePending = false;
	if (pros::competition::is_disabled()) {
		loweringPhase = LoweringPhase::idle;
		liftMotors.brake();
		return;
	}
	// lift_reset() owns reset movement; scoring must not overwrite its command.
	if (loweringPhase == LoweringPhase::resetting ||
	    controller.get_digital(liftResetButton) || controller.get_digital(liftUpButton) ||
	    controller.get_digital(liftManualDownButton)) return;

	const TravelStatus travel = liftTravelStatus(false);
	if (travel == TravelStatus::error) {
		cancelResetTilt();
		loweringPhase = LoweringPhase::idle;
		liftMotors.brake();
		return;
	}

	if (controller.get_digital(liftDownButton)) {
		cancelResetTilt();
		loweringPhase = LoweringPhase::manual;
		moveToLiftMinimum(liftDownSlowVelocityRpm);
		return;
	}

	if (loweringPhase == LoweringPhase::pickupReturning) {
		// Never let opening the claw send an elevated lift to the floor.
		// Recheck on every iteration, not only when the pickup initially starts.
		if (!liftWithinPickupBand()) {
			loweringPhase = LoweringPhase::idle;
		}
	}

	if (loweringPhase == LoweringPhase::manual) {
		// Release D-pad Down: open once, capture the release location, and hold the lift.
		// A same-tick manual claw click overrides the automatic open only;
		// preserve the normal scoring wait and pull-away requirement.
		if (!manualClawToggled) OpenClaw.extend();
		clawOpenedAtMs = pros::millis();
		const auto pose = chassis.getPose();
		releaseX = pose.x;
		releaseY = pose.y;
		pullAwayConfirmed = false;
		loweringPhase = LoweringPhase::waitingForPullAway;
	}

	if (loweringPhase == LoweringPhase::waitingForPullAway) {
		const auto pose = chassis.getPose();
		const bool driverMoving = std::abs(controller.get_analog(
		    pros::E_CONTROLLER_ANALOG_LEFT_Y)) > liftPullAwayJoystickDeadband;
		if (driverMoving && std::isfinite(releaseX) && std::isfinite(releaseY) &&
		    std::isfinite(pose.x) && std::isfinite(pose.y) &&
		    std::hypot(pose.x - releaseX, pose.y - releaseY) >= liftPullAwayDistanceIn) {
			pullAwayConfirmed = true;
		}
		if (pullAwayConfirmed && pros::millis() - clawOpenedAtMs >= liftClawOpenWaitMs) {
			loweringPhase = LoweringPhase::returningToBottom;
		}
	}

	if (loweringPhase == LoweringPhase::returningToBottom ||
	    loweringPhase == LoweringPhase::pickupReturning) {
		if (!moveToResetPosition()) {
			loweringPhase = LoweringPhase::idle;
		}
	} else if (!raiseToClosedFloorIfNeeded()) {
		// In particular, preserve HOLD throughout a scoring wait above the floor.
		liftMotors.brake();
	}
}

bool lift_move_to(double targetDegrees, int velocityRpm, unsigned int timeoutMs) {
	cancelResetTilt();
	loweringPhase = LoweringPhase::idle;
	manualClawTogglePending = false;
	liftMotors.set_brake_mode_all(pros::MotorBrake::hold);
	if (!std::isfinite(targetDegrees) || velocityRpm <= 0 || velocityRpm > 100 || timeoutMs == 0) {
		liftMotors.brake();
		return false;
	}
	const bool resetTarget = targetDegrees <= liftLowerLimitDegrees;
	if (resetTarget) resetMode = ResetMode::returningToZero;

	const unsigned int startedAt = pros::millis();
	unsigned int settledAt = 0;
	bool settling = false;
	while (pros::millis() - startedAt < timeoutMs) {
		if (pros::competition::is_disabled()) break;
		const auto positions = liftMotors.get_position_all();
		if (positions.empty()) break;
		const double target = std::clamp(targetDegrees,
		                                 resetTarget ? liftLowerLimitDegrees : liftEffectiveLowerLimit(),
		                                 static_cast<double>(liftUpperLimitDegrees));
		bool valid = true;
		bool reached = true;
		for (const double position : positions) {
			valid = valid && std::isfinite(position);
			reached = reached && std::fabs(target - position) <= liftMoveToleranceDegrees;
		}
		if (!valid) break;
		if (reached) {
			if (!settling) {
				settledAt = pros::millis();
				settling = true;
			}
			if (pros::millis() - settledAt >= liftMoveSettleMs) {
				if (resetTarget) resetMode = ResetMode::atZero;
				liftMotors.brake();
				return true;
			}
		} else {
			settling = false;
		}
		liftMotors.move_absolute(target, velocityRpm);
		pros::delay(10);
	}

	cancelResetTilt();
	liftMotors.brake();
	return false;
}
