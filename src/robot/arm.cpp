#include "robot/arm.hpp"

#include "robot/config.hpp"
#include "robot/hardware.hpp"
#include "robot/mechanisms.hpp"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"
#include <cmath>

std::atomic<std::int32_t> armTargetCentidegrees{0};
std::atomic<std::int32_t> armManualPower{0};
std::atomic<bool> armManualCoast{false};

// TUNE THESE: increase the hold power if the arm still rests below its target;
// decrease it if the arm steadily creeps upward.
namespace {
constexpr double armUpwardHoldPower = 15.0;
constexpr double armMinimumUpwardMovePower = 60.0;
constexpr double armMovePowerFadeDegrees = 12.0;
constexpr double armMaximumMovePower = 85.0;
constexpr double armDownwardCorrectionPerDegree = 4.0;
constexpr double armVelocityFilterWeight = 0.20;
constexpr double armVelocityDamping = 0.45;
constexpr double armDampingRangeDegrees = 35.0;
constexpr double armMaximumDampingPower = 35.0;

// The arm uses its curve across most of a normal move for gentler motion.
constexpr double armSlowdownDegrees = 100.0;

// Loop-carried state. Only armControlStep() and armDisable() touch these.
std::int32_t armPreviousTarget = 0;
std::int32_t armPreviousPosition = 0;
std::uint32_t armPreviousSampleTime = 0;
double armFilteredVelocity = 0.0;
bool armPidInitialized = false;

// Smooth S-curve for the arm. Its gentle slopes remove the high-power approach
// that was carrying the arm past its target through gearbox backlash.
double curvedArmOutput(double errorDegrees, double slowdownDegrees, double holdPower, double maximumPower,
                       double downwardCorrectionPerDegree) {
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
	return std::fmax(-maximumPower, holdPower + downwardCorrectionPerDegree * errorDegrees);
}
} // namespace

void setArmTarget(double degrees) {
	degrees = std::fmax(armMinimumDegrees, std::fmin(armMaximumDegrees, degrees));
	armTargetCentidegrees.store(degreesToCentidegrees(degrees));
}

void armDisable() {
	armMotor.move(0);
	armPidInitialized = false;
}

void armControlStep() {
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
		const bool pushingPastUpperLimit = manualArmPower > 0 && armPosition >= armMaximumCentidegrees;
		const bool pushingPastLowerLimit = manualArmPower < 0 && armPosition <= armMinimumCentidegrees;
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
				const double rawVelocity = ((armPosition - armPreviousPosition) / 100.0) * (1000.0 / sampleTimeMs);
				armFilteredVelocity += armVelocityFilterWeight * (rawVelocity - armFilteredVelocity);
				armPreviousPosition = armPosition;
				armPreviousSampleTime = armSampleTime;
			}
		}

		// Position zero receives no upward feedforward, so position 1 can rest at
		// the bottom. Every raised target gets gravity compensation.
		const double armHoldPower = armTarget > 0 ? armUpwardHoldPower : 0.0;
		double armOutput = curvedArmOutput(armError, armSlowdownDegrees, armHoldPower, armMaximumMovePower,
		                                   armDownwardCorrectionPerDegree);
		// A heavy arm needs more torque to begin lifting than it needs to remain
		// stationary. Apply a separate movement floor only while below target,
		// then fade it into the lower hold power over the final few degrees.
		if (armError > 0.0) {
			const double movePowerFraction = std::fmin(1.0, armError / armMovePowerFadeDegrees);
			const double minimumUpwardOutput =
			    armHoldPower + (armMinimumUpwardMovePower - armHoldPower) * movePowerFraction;
			armOutput = std::fmax(armOutput, minimumUpwardOutput);
		}
		// Remove power in proportion to measured arm speed as it approaches the
		// target. If the arm stalls, velocity becomes zero and the full movement
		// floor automatically returns, which is important for this heavy arm.
		const double dampingFraction = std::fmax(0.0, 1.0 - std::abs(armError) / armDampingRangeDegrees);
		double dampingPower = armVelocityDamping * armFilteredVelocity * dampingFraction;
		dampingPower = std::fmax(-armMaximumDampingPower, std::fmin(armMaximumDampingPower, dampingPower));
		armOutput -= dampingPower;
		// Automatic arm movement is intentionally capped below full power.
		armOutput = std::fmax(-127.0, std::fmin(127.0, armOutput));
		armMotor.move(static_cast<std::int32_t>(std::round(armOutput)));
	}
}

void armSpinToDegree(double degrees, std::uint32_t timeoutMs) {
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
		armMotor.set_brake_mode(armHoldingEnabled ? pros::MotorBrake::hold : pros::MotorBrake::coast);
		armManualCoast.store(false);
	}

	armManualPower.store(requestedPower);
}
