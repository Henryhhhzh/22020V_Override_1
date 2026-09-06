#include "autons/autons.hpp"

#include "22020Vlib/drive.hpp"
#include "robot/arm.hpp"
#include "robot/cascade.hpp"
#include "robot/claw.hpp"
#include "pros/rtos.hpp"

void simple_auton() {
	// Move the arm first and wait for it to reach 50 degrees.
	armSpinToDegree(100.0, 1000); // 6-second timeout
	cascadeSpinToDegree(500.0);
	armSpinToDegree(0.0, 1000);

	// TUNE THESE distances in inches and the direct motor power from 0 to 127.
	constexpr double backwardDistanceInches = -12.0;
	constexpr double forwardDistanceInches = 15.0;
	constexpr std::int32_t drivePower = 60;

	// Repeat: backward, stop, forward, stop — two times.
	for (int movement = 0; movement < 2; movement++) {
		simple_drive_distance(backwardDistanceInches, drivePower);
		simple_drive_distance(forwardDistanceInches, drivePower);
	}
	simple_drive_distance(-16.0, drivePower);
	simple_turn_degrees(-110.0, 50);
	simple_drive_backward_until_wall(50, 2000);
	cascadeSpinToDegree(350.0);
	toggleOpenClaw();
	pros::delay(100);
	simple_turn_degrees(200, 60);
	simple_drive_distance(20.0, 60);
	simple_turn_degrees(-100.0, 60);


}

	void example_auton() {
	//auton example there should be many of these

	}

		void example_auton2() {

		}
