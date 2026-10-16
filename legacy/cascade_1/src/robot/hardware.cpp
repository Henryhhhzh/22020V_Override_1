#include "robot/hardware.hpp"

// individual mechanism motors
pros::Motor armMotor(4); // normal direction; arm sensor remains reversed
pros::Motor cascadeMotor(-20);
pros::Motor intakeMotor(11);

// Double-acting pneumatic valves on the V5 brain's three-wire ADI ports.
// Both start retracted; extend() and retract() switch their two positions.
pros::adi::Pneumatics VariableClaw('H', false);
pros::adi::Pneumatics OpenClaw('F', false, true); // Inverted: retracted is physically closed.

// Auxiliary Rotation Sensor on port 5; not used for odometry.
pros::Rotation armRotationSensor(-5);

