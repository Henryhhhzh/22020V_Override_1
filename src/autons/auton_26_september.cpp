#include "autons/autons.hpp"

#include "22020Vlib/ramsete.hpp"

// Embedded from static/auton_26_september/, in numbered order.
// Dots and directory separators become underscores in the asset symbols.
ASSET(auton_26_september_auton_26_sep_1_ramsete_txt);
ASSET(auton_26_september_auton_26_sep_2_ramsete_txt);
ASSET(auton_26_september_auton_26_sep_3_ramsete_txt);
ASSET(auton_26_september_auton_26_sep_4_ramsete_txt);

void auton_26_september() {
	// Physically start at (-1.644, -61.437), facing heading 0 degrees.
	// Preserve odometry between segments and align to each next start heading.
	// Stop the sequence if a segment cannot be decoded.
	if (!follow_ramsete_asset(auton_26_september_auton_26_sep_1_ramsete_txt,
	                          RamseteStart::seedPose)) return;
	if (!follow_ramsete_asset(auton_26_september_auton_26_sep_2_ramsete_txt,
	                          RamseteStart::turnToStart)) return;
	if (!follow_ramsete_asset(auton_26_september_auton_26_sep_3_ramsete_txt,
	                          RamseteStart::turnToStart)) return;
	if (!follow_ramsete_asset(auton_26_september_auton_26_sep_4_ramsete_txt,
	                          RamseteStart::turnToStart)) return;
}
