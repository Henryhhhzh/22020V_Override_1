#include "autons/autons.hpp"

#include "22020Vlib/ramsete.hpp"

// Embedded from static/testing_curve_1_ramsete.txt.
// ASSET symbols replace each dot in the filename with an underscore.
ASSET(testing_curve_1_ramsete_txt);

void ramsete_auton_example() {
	// Start physically aligned with the export; seedPose sets odometry only.
	follow_ramsete_asset(testing_curve_1_ramsete_txt, RamseteStart::seedPose);
}
