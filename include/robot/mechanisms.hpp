#pragma once

#include <cstdint>

// Shared plumbing for the arm and cascade, plus the single task that owns both
// motors. The per-mechanism logic lives in robot/arm.hpp and robot/cascade.hpp.

std::int32_t degreesToCentidegrees(double degrees);

// The only task that commands the arm and cascade motors. It never times out:
// after reaching a target, it keeps correcting both mechanisms.
void mechanismPositionController();

// Preset positions, which move the arm and cascade together.
void moveToPosition(int positionNumber);
void togglePosition();
void goToPositionOne();
