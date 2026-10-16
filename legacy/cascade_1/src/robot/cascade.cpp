#include "robot/cascade.hpp"

#include "robot/config.hpp"
#include "robot/hardware.hpp"
#include "robot/mechanisms.hpp"
#include "pros/rtos.hpp"
#include <cmath>

std::atomic<std::int32_t> cascadeTargetCentidegrees{0};
std::atomic<std::int32_t> cascadeManualPower{0};

// TUNE THESE: increase the hold power if the cascade still rests below its
// target; decrease it if the cascade steadily creeps upward.
namespace {
constexpr double cascadeUpwardHoldPower = 25.0;

// The cascade keeps its own linear slowdown behavior, separate from the arm.
constexpr double cascadeSlowdownDegrees = 150.0;
constexpr double fullPowerKickMinimumErrorDegrees = 1.0;

// Loop-carried state. Only cascadeControlStep() and cascadeDisable() touch these.
std::int32_t cascadePreviousTarget = 0;
bool cascadePidInitialized = false;

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
} // namespace

void setCascadeTarget(double degrees) {
	degrees = std::fmax(cascadeMinimumDegrees, std::fmin(cascadeMaximumDegrees, degrees));
	cascadeTargetCentidegrees.store(degreesToCentidegrees(degrees));
}

void cascadeDisable() {
	cascadeMotor.move(0);
	cascadePidInitialized = false;
}

void cascadeControlStep() {
	const std::int32_t manualCascadePower = cascadeManualPower.load();
	const std::int32_t cascadeTarget = cascadeTargetCentidegrees.load();
	const double cascadePosition = cascadeMotor.get_position();
	if (!std::isfinite(cascadePosition)) {
		cascadeMotor.move(0);
		cascadePidInitialized = false;
	} else if (manualCascadePower != 0) {
		// Manual buttons temporarily own the cascade. Resetting this flag makes
		// the position controller restart cleanly when the buttons are released.
		const bool pushingPastUpperLimit = manualCascadePower > 0 && cascadePosition >= cascadeMaximumDegrees;
		const bool pushingPastLowerLimit = manualCascadePower < 0 && cascadePosition <= cascadeMinimumDegrees;
		cascadeMotor.move(pushingPastUpperLimit || pushingPastLowerLimit ? 0 : manualCascadePower);
		cascadePidInitialized = false;
	} else if (!cascadeHoldingEnabled) {
		// With no manual input, send zero power and let the coast brake mode act.
		cascadeMotor.move(0);
		cascadePidInitialized = false;
	} else {
		const double cascadeTargetDegrees = cascadeTarget / 100.0;
		const double cascadeError = cascadeTargetDegrees - cascadePosition;
		const bool targetChanged = !cascadePidInitialized || cascadeTarget != cascadePreviousTarget;
		if (targetChanged) {
			cascadePreviousTarget = cascadeTarget;
			cascadePidInitialized = true;
		}

		const double cascadeHoldPower = cascadeTarget > 0 ? cascadeUpwardHoldPower : 0.0;
		double cascadeOutput = linearMechanismOutput(cascadeError, cascadeSlowdownDegrees, cascadeHoldPower);
		if (targetChanged && std::abs(cascadeError) > fullPowerKickMinimumErrorDegrees) {
			cascadeOutput = std::copysign(127.0, cascadeError);
		}
		cascadeOutput = std::fmax(-127.0, std::fmin(127.0, cascadeOutput));
		cascadeMotor.move(static_cast<std::int32_t>(std::round(cascadeOutput)));
	}
}

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
