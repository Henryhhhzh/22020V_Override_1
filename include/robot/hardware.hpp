#pragma once

#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"
#include "robot/config.hpp"

// Every global object with a real constructor is DEFINED in src/robot/hardware.cpp
// and only declared here. Keeping all of those definitions in one translation
// unit is what guarantees they are constructed in the written order, which the
// chassis depends on: it is built from the drivetrain, controller settings, and
// sensors declared above it.

extern pros::Controller controller;

// motor groups
extern pros::MotorGroup leftMotors;
extern pros::MotorGroup rightMotors;
extern pros::MotorGroup liftMotors;
extern pros::MotorGroup intakeMotors;

// Main claw pneumatic on ADI H.
extern pros::adi::Pneumatics OpenClaw;

// Claw tilt pneumatic on ADI G; extended is the default lift position.
extern pros::adi::Pneumatics ClawTilt;

// Inertial sensor
extern pros::Imu imu;

// tracking wheels
extern pros::Rotation horizontalEnc;
extern lemlib::TrackingWheel horizontal;

// chassis configuration
extern lemlib::Drivetrain drivetrain;
extern lemlib::ControllerSettings linearController;
extern lemlib::ControllerSettings angularController;
extern lemlib::OdomSensors sensors;
extern lemlib::ExpoDriveCurve throttleCurve;
extern lemlib::ExpoDriveCurve steerCurve;
extern lemlib::Chassis chassis;

// DSR physical sensor mounting settings
extern DsrSensorConfig dsrFrontSensor;
extern DsrSensorConfig dsrRightSensor;
extern DsrSensorConfig dsrBackSensor;
extern DsrSensorConfig dsrLeftSensor;
