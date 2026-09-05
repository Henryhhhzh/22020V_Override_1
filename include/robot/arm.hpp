#pragma once

#include <atomic>
#include <cstdint>

// Targets are stored in centidegrees so changing a target is atomic on the V5
// brain. Anything outside this module sets a target and returns immediately;
// only armControlStep() ever commands armMotor.
extern std::atomic<std::int32_t> armTargetCentidegrees;
extern std::atomic<std::int32_t> armManualPower;
extern std::atomic<bool> armManualCoast;

void setArmTarget(double degrees);

// Set an absolute arm target and wait until the port-5 Rotation Sensor settles.
// A timeout of 0 keeps the original unlimited wait behavior.
void armSpinToDegree(double degrees, std::uint32_t timeoutMs = 0);

void handleManualArmControl();

// One iteration of the arm half of the mechanism control loop. Called only by
// mechanismPositionController().
void armControlStep();
void armDisable();
