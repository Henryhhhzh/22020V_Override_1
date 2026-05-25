#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include <cmath>

pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup rightMotors({1, 7, 15},
	                            pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup leftMotors({-3, -4, -5}, pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)

enum class RobotDsrSensor {
	top,
	right,
	bottom,
	left
};

enum class DsrWall {
	top,
	right,
	bottom,
	left
};

struct DsrSensorConfig {
	float heading_offset; // direction the sensor points compared to robot front, in degrees
	float x_offset; // sensor-local forward offset from tracking point to sensor lens, in inches
	float y_offset; // sensor-local right-side offset from tracking point to sensor lens, in inches

	DsrSensorConfig(float heading_offset, float x_offset, float y_offset)
	    : heading_offset(heading_offset), x_offset(x_offset), y_offset(y_offset) {}
};

// individual motors and pistons
//motrs and pistons here
//individual motors and pistons


// Inertial Sensor on port ?
pros::Imu imu(8);


// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
pros::Rotation horizontalEnc(-13);
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
pros::Rotation verticalEnc(-2);
// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_2, -4.523);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_2, -0.5);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              10.1, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              450, // drivetrain rpm is 360
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
lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
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

// DSR wall locations
constexpr float dsrTopWallY = 72.0; // top wall Y coordinate, in inches
constexpr float dsrRightWallX = 72.0; // right wall X coordinate, in inches
constexpr float dsrBottomWallY = -72.0; // bottom wall Y coordinate, in inches
constexpr float dsrLeftWallX = -72.0; // left wall X coordinate, in inches

// DSR reading limits
constexpr float dsrMinValidDistance = 0.5; // minimum valid distance sensor reading, in inches
constexpr float dsrMaxValidDistance = 100.0; // maximum valid distance sensor reading, in inches

// DSR sensor offsets are measured from the tracking point to the distance sensor lens.
// These are sensor-local offsets, so they are different for each physical sensor:
// x_offset = forward/back along the direction that sensor points. Positive is outward.
// y_offset = side offset to that sensor's right. Negative is to that sensor's left.
// Example: top sensor 5" in front of tracking point and 1" to robot right -> (0, 5, 1).
// Example: right sensor 4" to robot right and 0.5" toward robot front -> (90, 4, -0.5).
// THE CORDS ARE NOT THE UNIVERSAL PLANE BUT CENTERERED ON THE SENSOR IN QUESTION
// DSR top physical sensor settings
DsrSensorConfig dsrTopSensor(0, // heading offset from robot front, in degrees
	                             0, // x_offset: positive toward robot front/top, in inches
	                             0 // y_offset: positive toward robot right, in inches
);

// DSR right physical sensor settings
DsrSensorConfig dsrRightSensor(90, // heading offset from robot front, in degrees
	                               0, // x_offset: positive toward robot right, in inches
	                               0 // y_offset: positive toward robot back/bottom, in inches
);

// DSR bottom physical sensor settings
DsrSensorConfig dsrBottomSensor(180, // heading offset from robot front, in degrees
	                                0, // x_offset: positive toward robot back/bottom, in inches
	                                0 // y_offset: positive toward robot left, in inches
);

// DSR left physical sensor settings
DsrSensorConfig dsrLeftSensor(270, // heading offset from robot front, in degrees
	                              0, // x_offset: positive toward robot left, in inches
	                              0 // y_offset: positive toward robot front/top, in inches
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

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
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

//robot functions driver and auton

//DISTANCE SENSOR RESET FUNCTIONS (priotrity use the top one that is available, if not use the next one down, etc.)

	float dsr_deg_to_rad(float degrees) {
		return degrees * 3.1415926535 / 180.0;
	}

	DsrSensorConfig dsr_get_sensor_config(RobotDsrSensor sensor) {
		switch (sensor) {
			case RobotDsrSensor::top:
				return dsrTopSensor;
			case RobotDsrSensor::right:
				return dsrRightSensor;
			case RobotDsrSensor::bottom:
				return dsrBottomSensor;
			case RobotDsrSensor::left:
				return dsrLeftSensor;
		}

		return {0, 0, 0};
	}

	float dsr_read_sensor_inches(RobotDsrSensor sensor) {
		// TODO: return the selected physical distance sensor's reading in inches.
		(void)sensor;
		return -1;
	}

	float dsr_wall_angle(DsrWall wall) {
		switch (wall) {
			case DsrWall::top:
				return 0;
			case DsrWall::right:
				return 90;
			case DsrWall::bottom:
				return 180;
			case DsrWall::left:
				return 270;
		}

		return 0;
	}

	float dsr_tracking_point_distance_to_wall(RobotDsrSensor sensor, DsrWall wall, float heading) {
		float sensor_distance = dsr_read_sensor_inches(sensor);
		if (sensor_distance < dsrMinValidDistance || sensor_distance > dsrMaxValidDistance) {
			return -1;
		}

		DsrSensorConfig config = dsr_get_sensor_config(sensor);
		float theta = dsr_deg_to_rad((heading + config.heading_offset) - dsr_wall_angle(wall));
		float blue = std::cos(theta) * sensor_distance;
		float purple = std::cos(theta) * config.x_offset;
		float green = std::sin(theta) * config.y_offset * -1;

		return blue + purple + green;
	}

	void dsr_add_coordinate_sample(float& x_sum, int& x_count, float& y_sum, int& y_count,
	                               RobotDsrSensor sensor, DsrWall wall, float heading) {
		float distance_to_wall = dsr_tracking_point_distance_to_wall(sensor, wall, heading);
		if (distance_to_wall < 0) {
			return;
		}

		switch (wall) {
			case DsrWall::top:
				y_sum += dsrTopWallY - distance_to_wall;
				y_count++;
				break;
			case DsrWall::bottom:
				y_sum += dsrBottomWallY + distance_to_wall;
				y_count++;
				break;
			case DsrWall::right:
				x_sum += dsrRightWallX - distance_to_wall;
				x_count++;
				break;
			case DsrWall::left:
				x_sum += dsrLeftWallX + distance_to_wall;
				x_count++;
				break;
		}
	}

	/**
	 * Runs during auto to reset odom with any combination of distance sensors.
	 * These booleans are field-facing directions, not physical sensor names.
	 */
	void dsr(bool top_facing, bool left_facing, bool right_facing, bool bottom_facing){
		lemlib::Pose pose = chassis.getPose();
		float heading = pose.theta;
		RobotDsrSensor top_facing_sensor = RobotDsrSensor::top;
		RobotDsrSensor right_facing_sensor = RobotDsrSensor::right;
		RobotDsrSensor bottom_facing_sensor = RobotDsrSensor::bottom;
		RobotDsrSensor left_facing_sensor = RobotDsrSensor::left;

		while (heading < 0) heading += 360;
		while (heading >= 360) heading -= 360;

		if (heading >= 315.5 || heading < 45.5) {
			// Nearest field direction: top / heading 1
			top_facing_sensor = RobotDsrSensor::top;
			right_facing_sensor = RobotDsrSensor::right;
			bottom_facing_sensor = RobotDsrSensor::bottom;
			left_facing_sensor = RobotDsrSensor::left;
		} else if (heading < 135) {
			// Nearest field direction: right / heading 90
			top_facing_sensor = RobotDsrSensor::left;
			right_facing_sensor = RobotDsrSensor::top;
			bottom_facing_sensor = RobotDsrSensor::right;
			left_facing_sensor = RobotDsrSensor::bottom;
		} else if (heading < 225) {
			// Nearest field direction: bottom / heading 180
			top_facing_sensor = RobotDsrSensor::bottom;
			right_facing_sensor = RobotDsrSensor::left;
			bottom_facing_sensor = RobotDsrSensor::top;
			left_facing_sensor = RobotDsrSensor::right;
		} else {
			// Nearest field direction: left / heading 270
			top_facing_sensor = RobotDsrSensor::right;
			right_facing_sensor = RobotDsrSensor::bottom;
			bottom_facing_sensor = RobotDsrSensor::left;
			left_facing_sensor = RobotDsrSensor::top;
		}

		float x_sum = 0;
		float y_sum = 0;
		int x_count = 0;
		int y_count = 0;

		if (top_facing) {
			dsr_add_coordinate_sample(x_sum, x_count, y_sum, y_count, top_facing_sensor, DsrWall::top, heading);
		}
		if (bottom_facing) {
			dsr_add_coordinate_sample(x_sum, x_count, y_sum, y_count, bottom_facing_sensor, DsrWall::bottom, heading);
		}
		if (left_facing) {
			dsr_add_coordinate_sample(x_sum, x_count, y_sum, y_count, left_facing_sensor, DsrWall::left, heading);
		}
		if (right_facing) {
			dsr_add_coordinate_sample(x_sum, x_count, y_sum, y_count, right_facing_sensor, DsrWall::right, heading);
		}

		float new_x = x_count > 0 ? x_sum / x_count : pose.x;
		float new_y = y_count > 0 ? y_sum / y_count : pose.y;
		if (x_count > 0 || y_count > 0) {
			chassis.setPose(new_x, new_y, pose.theta);
		}
	}

//robot functions driver and auton end

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
 * all different autons will be here
 * put the autons you want to run in the autonomous function at the bottom
 */

	void example_auton() {
	//auton example there should be many of these

	}

	void example_auton2() {

	}

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
	example_auton();
    //the auton you want to run
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
        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        //chassis.tank(leftY, rightY);
        if (controller.get_digital_new_press(DIGITAL_A)){
            chassis.setPose(0,0,0);
            chassis.tank(-115, 125);
            pros::delay(400);
            chassis.turnToHeading(345, 300, {.minSpeed=110});
            std::cout<< chassis.getPose().theta;
            // chassis.tank(120, -120);
            // pros::delay(250);
        }
        
        chassis.arcade(leftY, rightX);
        // delay to save resources
        pros::delay(50);
    }
}
