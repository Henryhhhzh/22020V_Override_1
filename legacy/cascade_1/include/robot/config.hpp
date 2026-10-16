#pragma once

#include "pros/misc.h"
#include <cstdint>

// One mechanism position is always written as {arm degrees, cascade degrees}.
struct PositionTargets {
	double armDegrees;
	double cascadeDegrees;
};

// TUNE THESE: target reached immediately after the sensors are zeroed at startup.
// Position 1 and the reset button use this same base position.
constexpr PositionTargets startupBasePosition = {0.0, 125.0};

// true: active position correction + hold brake
// false: no position correction + coast brake (manual control only)
constexpr bool armHoldingEnabled = true;
constexpr bool cascadeHoldingEnabled = true;

// Mechanism tolerances, loop timing, and travel limits
constexpr double cascadeTargetToleranceDegrees = 25.0;
constexpr std::int32_t armTargetToleranceCentidegrees = 500;
constexpr std::uint32_t mechanismSettleTimeMs = 100;
constexpr std::uint32_t mechanismLoopMs = 10;
constexpr double armMinimumDegrees = 0.0;
constexpr double armMaximumDegrees = 120.0;
constexpr double cascadeMinimumDegrees = 0.0;
constexpr double cascadeMaximumDegrees = 1500.0;
constexpr std::int32_t armMinimumCentidegrees = 0;
constexpr std::int32_t armMaximumCentidegrees = 10000;
constexpr std::int32_t armZeroRestBandCentidegrees = 200;

// Array index 0 is position 1; array index 5 is position 6.
constexpr PositionTargets positions[6] = {
	{startupBasePosition.armDegrees, startupBasePosition.cascadeDegrees}, // position 1 / reset
	{0, 1200.0}, // position 3
	{110.0, 0.0},   // position 4
	{110.0, 1000.0},// position 5
	{110.0, 1000.0},// position 6
	{110.0, 1000.0}

};

// Driver control button map
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
constexpr auto outtakeButton = pros::E_CONTROLLER_DIGITAL_B;
constexpr std::int32_t outtakeSpeed = -127;
constexpr double clawCloseCascadeLiftDegrees = 150.0;
