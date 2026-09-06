#include "autons/autons.hpp"

#include "22020Vlib/ramsete.hpp"
#include "robot/hardware.hpp"

	// Example: a 24-inch forward move written in the planner's row format.
// Replace these rows with your own export. Each row is:
//   { time_s, x_in, y_in, theta_rad, v_ips, omega_radps }
static const RamsetePoint kExampleRamsetePath[] = {
	{0.0, 0.0, 0.0, 1.5708, 0.0, 0.0},
	{0.2, 0.0, 1.2, 1.5708, 12.0, 0.0},
	{0.4, 0.0, 4.8, 1.5708, 24.0, 0.0},
	{0.6, 0.0, 9.6, 1.5708, 24.0, 0.0},
	{0.8, 0.0, 14.4, 1.5708, 24.0, 0.0},
	{1.0, 0.0, 19.2, 1.5708, 24.0, 0.0},
	{1.2, 0.0, 22.8, 1.5708, 12.0, 0.0},
	{1.4, 0.0, 24.0, 1.5708, 0.0, 0.0},
};

void ramsete_auton_example() {
	int count = sizeof(kExampleRamsetePath) / sizeof(kExampleRamsetePath[0]);
	// Seed odom to the path's first point, then follow it.
	chassis.setPose(0, 0, 0);
	follow_ramsete(kExampleRamsetePath, count);
}
