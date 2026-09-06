#include "autons/autons.hpp"

#include "22020Vlib/dsr.hpp"
#include "22020Vlib/ramsete.hpp"
#include "lemlib/asset.hpp"

// The five segments this routine drives. ASSET() embeds a file from static/
// into the binary; the symbol is the filename with every '.' replaced by '_'.
ASSET(bot1_auton1_part1_ramsete_txt);
ASSET(bot1_auton1_part2_ramsete_txt);
ASSET(bot1_auton1_part3_ramsete_txt);
ASSET(bot1_auton1_part4_ramsete_txt);
ASSET(bot1_auton1_part5_ramsete_txt);

// Bot 1's first routine, driven from the five planner exports in static/.
//
// The planner writes each segment as its own file with its own start heading and
// no turn between them, so the joins have to be handled here. Headings below are
// LemLib degrees (0 = +Y, clockwise positive):
//
//   part 1  (0.1, -63.8) h180 -> (-16.7, -46.9) h89    reverse, curves left
//   part 2  (-16.7, -46.9) h90 -> (0.1, -46.9) h90     forward
//   part 3  (0.1, -46.9) h0    -> (0.1, -66.0) h0      reverse   <- 90 deg turn in
//   part 4  (0.1, -66.0) h180  -> (0.1, -23.4) h180    reverse   <- 180 deg turn in
//   part 5  (0.1, -23.4) h179  -> (-0.1, -4.8) h179    reverse
//
// Only two joins actually need a turn. Parts 1->2 and 4->5 already line up to
// within a degree, so turning there would just waste time and add error.
void bot1_auton1() {
	follow_ramsete_asset(bot1_auton1_part1_ramsete_txt, RamseteStart::seedPose);
	follow_ramsete_asset(bot1_auton1_part2_ramsete_txt, RamseteStart::asIs);
	follow_ramsete_asset(bot1_auton1_part3_ramsete_txt, RamseteStart::turnToStart);
	follow_ramsete_asset(bot1_auton1_part4_ramsete_txt, RamseteStart::turnToStart);
	follow_ramsete_asset(bot1_auton1_part5_ramsete_txt, RamseteStart::asIs);
}
