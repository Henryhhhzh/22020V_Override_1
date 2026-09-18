#include "autons/autons.hpp"

#include "22020Vlib/ramsete.hpp"

// Two numbered segments embedded from static/circle_test/.
ASSET(circle_test_circile_test_1_ramsete_txt);
ASSET(circle_test_circile_test_2_ramsete_txt);

void ramsete_auton_example() {
	// Physically align with the first path; seedPose sets odometry only.
	if (!follow_ramsete_asset(circle_test_circile_test_1_ramsete_txt,
	                          RamseteStart::seedPose)) return;
	if (!follow_ramsete_asset(circle_test_circile_test_2_ramsete_txt,
	                          RamseteStart::turnToStart)) return;
}
