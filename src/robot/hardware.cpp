#include "robot/hardware.hpp"

// Definition order in this file is load-bearing. C++ only guarantees that
// globals are constructed top to bottom within a single translation unit, and
// the chassis at the bottom is constructed from the drivetrain, both controller
// settings, the odom sensors, and both drive curves. Keep every one of those
// definitions in this file, above the chassis.

pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup leftMotors({1, -2, -3}, pros::MotorGearset::blue); // left drive motors, reversed
pros::MotorGroup rightMotors({-10, 9, 8}, pros::MotorGearset::blue); // right drive motors

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

// Inertial sensor on confirmed port 19.
pros::Imu imu(19);

// tracking wheels
// Horizontal Rotation Sensor on confirmed port 7. Reverse the reported
// left-increasing raw reading so rightward travel increases the odom reading.
pros::Rotation horizontalEnc(-7);
// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_2, -4.523);

// drivetrain settings
// Geometry comes from robot/config.hpp so the Ramsete follower and the simple_*
// encoder moves cannot disagree with what odometry is scaled to.
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
	                              kTrackWidthIn, // center of middle left wheel to middle right wheel, in inches
                              kDriveWheelDiameterIn, // new 2.75" omnis
                              kDriveWheelRpm, // drivetrain wheel rpm
                              1 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(5.4, // proportional gain (kP)
                                            0.2, // integral gain (kI)
                                            3.3, // derivative gain (kD)
                                            3, // anti windup
                                            .8, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            2, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(2, // proportional gain (kP)
                                             0.1, // integral gain (kI)
                                             13, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
// TEMPORARY: forward tracking runs off the drive motor encoders instead of the
// vertical tracking wheel. Both vertical slots are nullptr, so chassis.calibrate()
// builds one tracking wheel per drive side from the drivetrain settings above
// (2.75" wheel, 450 rpm, +-trackWidth/2 offset). This is why the drivetrain's
// wheel diameter and rpm now have to be exactly right: they set the odom scale.
// Motor encoders read wheel slip as real distance, so expect drift under pushing
// and after hard stops. To restore a vertical tracking wheel, define it on its
// confirmed sensor port and pass it as the first argument.
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel 1 -> left drive motors
                            nullptr, // vertical tracking wheel 2 -> right drive motors
                            &horizontal, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
	                                  10, // minimum output where drivetrain will move out of 127
	                                  1.015 // expo curve gain
);

// DSR sensor offsets are measured from the tracking point to the distance sensor lens.
// These are sensor-local offsets, so they are different for each physical sensor:
// x_offset = forward/back along the direction that sensor points. Positive is outward.
// y_offset = side offset to that sensor's right. Negative is to that sensor's left.
// Example: front sensor 5" in front of tracking point and 1" to robot right -> (0, 5, 1).
// Example: right sensor 4" to robot right and 0.5" toward robot front -> (90, 4, -0.5).
// These offset coordinates are sensor-local, not field-global.
// DSR front physical sensor settings
DsrSensorConfig dsrFrontSensor(0, // heading offset from robot front, in degrees
	                               0, // x_offset: positive toward robot front, in inches
	                               0 // y_offset: positive toward robot right, in inches
);

// DSR right physical sensor settings
DsrSensorConfig dsrRightSensor(90, // heading offset from robot front, in degrees
	                               0, // x_offset: positive toward robot right, in inches
	                               0 // y_offset: positive toward robot back, in inches
);

// DSR back physical sensor settings
DsrSensorConfig dsrBackSensor(180, // heading offset from robot front, in degrees
	                              0, // x_offset: positive toward robot back, in inches
	                              0 // y_offset: positive toward robot left, in inches
);

// DSR left physical sensor settings
DsrSensorConfig dsrLeftSensor(270, // heading offset from robot front, in degrees
	                              0, // x_offset: positive toward robot left, in inches
	                              0 // y_offset: positive toward robot front, in inches
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);
