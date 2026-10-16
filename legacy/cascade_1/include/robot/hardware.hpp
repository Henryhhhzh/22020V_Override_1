#pragma once

#include "pros/adi.hpp"
#include "pros/misc.hpp"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"
#include "robot/config.hpp"

// Shared driver controller remains in the active hardware module.
extern pros::Controller controller;

// individual mechanism motors
extern pros::Motor armMotor;
extern pros::Motor cascadeMotor;
extern pros::Motor intakeMotor;

// Double-acting pneumatic valves on the V5 brain's three-wire ADI ports.
extern pros::adi::Pneumatics VariableClaw;
extern pros::adi::Pneumatics OpenClaw;

// Auxiliary Rotation Sensor on port 5; not used for odometry.
extern pros::Rotation armRotationSensor;

