#include "main.h"
#include "autons/autons.hpp"
#include "robot/claw.hpp"
#include "robot/hardware.hpp"
#include "robot/intake.hpp"
#include "robot/lift.hpp"
#include <cmath>

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	restoreClawTilt();
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	// Nothing that reads chassis.getPose() works until this runs: it calibrates
	// the IMU, resets the tracking wheels, and starts LemLib's odometry task.
	// With both vertical slots left null it also builds the forward tracking
	// wheels from the drive motor groups. Blocks for roughly three seconds, and
	// the robot must be still for all of it.
	chassis.calibrate();

	// PROS motors default to coast, and MotorGroup::brake() only ever applies
	// whatever brake mode is already set. Without this the Ramsete follower's
	// brake() at the end of each segment coasts, and the robot drifts past the
	// endpoint before the next segment picks up from it.
	leftMotors.set_brake_mode_all(pros::MotorBrake::brake);
	rightMotors.set_brake_mode_all(pros::MotorBrake::brake);

	// Close around the preload and record the closed logical state. The first
	// R2 click will OPEN it. Initialize at zero, then lift to the closed-claw
	// clearance automatically when driver control is enabled.
	OpenClaw.retract();

	// Start the program with the lift physically at its bottom (0-degree) position.
	// This defines encoder zero; it does not move or automatically home the lift.
	liftMotors.set_encoder_units_all(pros::MotorUnits::degrees);
	liftMotors.tare_position_all();
	liftMotors.set_brake_mode_all(pros::MotorBrake::hold);
	liftMotors.brake();

	// Brain line 3 continuously shows the average motor encoder angle, including
	// while stopped. These are motor degrees, before any external lift gearing.
	static pros::Task liftDisplayTask([] {
		while (true) {
			const auto positions = liftMotors.get_position_all();
			double sum = 0.0;
			bool valid = !positions.empty();
			for (const double position : positions) {
				valid = valid && std::isfinite(position);
				sum += position;
			}
			if (valid) {
				pros::lcd::print(3, "Lift: %.1f motor deg", sum / positions.size());
			} else {
				pros::lcd::set_text(3, "Lift: encoder error");
			}
			pros::delay(100);
		}
	}, "Lift display");
	(void)liftDisplayTask;

	pros::lcd::register_btn1_cb(on_center_button);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
	// Cancel any pending lift return when competition control disables the robot.
	lift_reset();
	lift_up();
	lift_manual_down();
	lift_down();
}

// Where the rest of the code went:
//   22020Vlib/ramsete.{hpp,cpp}  trajectory following and path loading
//   22020Vlib/dsr.{hpp,cpp}      distance sensor odometry reset
//   22020Vlib/drive.{hpp,cpp}    simple_drive_distance / turn / until_wall
//   src/autons/                  one file per routine, listed in autons/autons.hpp
//   robot/config.hpp             every tunable number, including drivetrain geometry
//   legacy/cascade_1/            retired mechanism ports, controls, and routines

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}


/**
 * THE AUTON YOU WANT TO RUN GOES IN THE AUTONOMOUS FUNCTION AT THE BOTTOM
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
	// The current test run uses the circle-test Ramsete paths.
	ramsete_auton_example();
	// To select the lift-only routine instead, call:
	//   simple_auton();
	// To run the five planner exports in static/ instead, call:
	//   bot1_auton1();
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
    // controller
    // loop to continuously update motors

    while (true) {
		handleClawControl();
		lift_reset();
		lift_up();
		lift_manual_down();
		lift_down();
		intake_in();
		intake_out();

		// get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        //chassis.tank(leftY, rightY);
		// A-button drivetrain test disabled.
		// if (controller.get_digital_new_press(DIGITAL_A)) {
		// 	chassis.setPose(0, 0, 0);
		// 	chassis.tank(-115, 125);
		// 	pros::delay(400);
		// 	chassis.turnToHeading(345, 300, {.minSpeed = 110});
		// 	std::cout << chassis.getPose().theta;
		// }
        
        chassis.arcade(leftY, rightX);
        // delay to save resources
        pros::delay(50);
    }
}
