#pragma once

#include <cstdint>

// Encoder-only drive primitives. These read the integrated motor encoders
// directly and command motor power: no odometry, no DSR, no LemLib motion, no
// IMU. That makes them usable before odom is trustworthy and immune to a
// tracking wheel problem, at the cost of being open-loop about where the robot
// actually ends up. For anything that has to land on a field coordinate, use a
// Ramsete path or a LemLib motion instead.
//
// All three block until they finish or time out, then brake.

// Straight movement. Negative inches drive backward. Power is 0 to 127.
// Gives up after 3 seconds regardless of distance covered.
void simple_drive_distance(double distanceInches, std::int32_t power);

// Relative point turn. Positive degrees turn right, negative turn left.
// Does not read or target an IMU heading, so error accumulates across turns.
void simple_turn_degrees(double turnDegrees, std::int32_t power);

// Drive backward until the drivetrain is loaded and nearly stopped, as when
// contacting a wall. Useful for squaring against a field wall to kill
// accumulated heading error before a path starts.
void simple_drive_backward_until_wall(std::int32_t power, std::uint32_t timeoutMs = 4000);
