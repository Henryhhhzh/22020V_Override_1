#pragma once

#include "lemlib/asset.hpp"

// Ramsete trajectory follower for planner-exported paths.
//
// A trajectory is a time-indexed list of where the robot should be and how fast
// it should be going. The follower reads the current odom pose every 10 ms,
// compares it to where the trajectory says it should be right now, and blends
// the path's own velocity with a correction back onto it.
//
// Paths are exported from path.jerryio in "LemLib Ramsete Beta" format and live
// in static/, which the PROS build embeds into the binary. Use ASSET() in the
// auton file that needs a path, then hand it to follow_ramsete_asset().

// One row of a planner-exported trajectory.
// theta is a math-frame angle in radians: 0 = +X (field right), counter-clockwise
// positive, exactly as the planner writes theta_rad. v is inches/second, omega is
// radians/second. x and y are LemLib odom inches (x = right, y = forward).
struct RamsetePoint {
	float t; // time_s
	float x; // x_in  (field X, right)
	float y; // y_in  (field Y, forward)
	float theta; // theta_rad (math frame)
	float v; // v_ips
	float omega; // omega_radps
};

// Largest trajectory that can be decoded. At the planner's 0.02 s dt this is
// 6.4 seconds of path; the longest current segment is 84 rows.
constexpr int kRamseteMaxPoints = 320;

// How the robot should get onto the start of a segment.
enum class RamseteStart {
	seedPose, // Trust the export and snap odom to its first row. First segment only.
	turnToStart, // Odom is good, but this segment starts facing somewhere else.
	asIs // The previous segment already left the robot on this heading.
};

// Decode an embedded export, repair its heading columns, enter it, and follow it.
// Returns false if the asset did not decode, which means the segment was skipped
// rather than driven blind.
bool follow_ramsete_asset(const asset& file, RamseteStart start);

// Follow a trajectory already in memory. Blocks until the trajectory time
// elapses, then brakes. Odom must already be seeded to the path's first point.
void follow_ramsete(const RamsetePoint* traj, int count);

// Read an embedded export into out[]. Rows are "t, x, y, theta, v, omega";
// '#' comment lines, blank lines, and the trailing "endData" marker are skipped.
// Returns how many rows were decoded.
int ramsete_parse(const asset& file, RamsetePoint* out, int capacity);

// Rebuild the theta and omega columns from the path tangent. The planner's own
// values are unusable as feedforward; see the comment on the definition.
// follow_ramsete_asset() already does this, so call it directly only when
// inspecting a trajectory by hand.
void ramsete_rebuild_heading(RamsetePoint* traj, int count);

// LemLib pose.theta is degrees, 0 = +Y (forward), clockwise positive.
// The planner's theta is a math angle, 0 = +X (right), counter-clockwise positive.
// Useful for working out where to physically place the robot before a segment:
// convert row one's theta_rad and that is the heading to start on.
float ramsete_heading_to_math_rad(float heading_deg);
float ramsete_math_rad_to_heading(float math_rad);
