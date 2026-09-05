#pragma once

#include <atomic>
#include <cstdint>

// Targets are stored in centidegrees so changing a target is atomic on the V5
// brain. Anything outside this module sets a target and returns immediately;
// only cascadeControlStep() ever commands cascadeMotor.
extern std::atomic<std::int32_t> cascadeTargetCentidegrees;
extern std::atomic<std::int32_t> cascadeManualPower;

void setCascadeTarget(double degrees);

// Set an absolute cascade target and wait until it remains within 25 degrees.
// During calibration, correction is disabled, so this records the target and
// returns without trying to move or hold the cascade.
void cascadeSpinToDegree(double degrees);

void handleManualCascadeControl();

// One iteration of the cascade half of the mechanism control loop. Called only
// by mechanismPositionController().
void cascadeControlStep();
void cascadeDisable();
