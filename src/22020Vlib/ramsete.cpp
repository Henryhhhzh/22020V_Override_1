#include "22020Vlib/ramsete.hpp"

#include "robot/config.hpp"
#include "robot/hardware.hpp"
#include "pros/rtos.hpp"
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace {

// The V5 controller link is slow: PROS drops writes closer together than this,
// and every attempt still takes the controller port mutex. Calling it at the
// full control rate added jitter to the loop for four updates out of five.
constexpr uint32_t kRamseteDisplayMs = 50;

// Fixed 15-character row: A-06 L+01 H-019. Position is rounded inches; heading
// is rounded degrees. Show overflow/invalid readings as question marks.
//
// A and L are robot-frame, not field-frame: A is along-track (ahead of or behind
// the path along the robot's own heading) and L is lateral (off to its side).
// Field X and Y would mean something different at every heading -- on a path
// that sweeps 180 degrees they swap roles halfway through -- so they cannot be
// read as a diagnosis. These two always can: A is follower lag, L is odometry.
// force bypasses the rate gate for the one final snapshot at the end of a path.
void display_ramsete_errors(float x, float y, float heading, bool force = false) {
	static uint32_t last_print_ms = 0;
	const uint32_t now = pros::millis();
	if (!force && now - last_print_ms < kRamseteDisplayMs) return;
	last_print_ms = now;

	char fields[3][5];
	const float values[] = {x, y, heading};
	for (int i = 0; i < 3; ++i) {
		const float rounded = std::round(values[i]);
		const int limit = i == 2 ? 180 : 99;
		if (!std::isfinite(rounded) || std::fabs(rounded) > limit) {
			std::snprintf(fields[i], sizeof(fields[i]), "%s", i == 2 ? "????" : "???");
		} else {
			std::snprintf(fields[i], sizeof(fields[i]), i == 2 ? "%+04d" : "%+03d",
			              static_cast<int>(rounded));
		}
	}
	controller.print(0, 0, "A%s L%s H%s", fields[0], fields[1], fields[2]);
}

// Ramsete b has units rad^2 / distance^2, and the literature quotes it in
// rad^2/m^2 with 2.0 as the usual starting point. Trajectory velocities and pose
// errors here are in inches, so the constant has to be converted. Note that the
// leading value below is 20.0, not 2.0: this is ten times the textbook starting
// point. Retune by changing that leading number, not the conversion.
constexpr float kMetersPerInch = 0.0254;
constexpr float kRamseteB = 2.0 * kMetersPerInch * kMetersPerInch;
constexpr float kRamseteZeta = 0.9;

constexpr float kRamsetePi = 3.14159265358979;
constexpr int kRamseteLoopMs = 10; // 100 Hz control loop
constexpr int kRamseteMaxLineChars = 128;

// The Ramsete gain k is proportional to the trajectory's own speed, so it falls
// to zero wherever the trajectory is stationary: every segment's first and last
// rows, and the whole settle phase below. That is exactly where the correction
// is needed, so floor the speed that feeds the gain. This does not change the
// commanded velocity, only how hard the controller pulls back onto the path.
constexpr float kRamseteMinGainSpeedIps = 10.0;

// How close the robot has to get to the final row before a segment counts as
// finished, and how long it may keep trying. The drivetrain always lags a
// time-indexed trajectory, so stopping the moment the clock runs out ends every
// segment short and hands that error to the next one.
//
// The tolerance is measured along the robot's own heading, not as a straight
// line distance. A differential drive parked on a heading can drive forward and
// it can rotate, but it cannot move sideways, and the one term in the control
// law that corrects sideways error is multiplied by the trajectory velocity --
// which is zero here. So a segment that ends with a lateral offset can never
// close it, and testing a straight line distance would only burn the timeout
// every time. Along-track and heading are exactly what the settle can fix.
constexpr float kRamseteSettleAlongTrackIn = 1.0;
constexpr float kRamseteSettleHeadingDeg = 0.5;
constexpr float kRamseteSettleTimeoutS = 0.5;

// How long turnToStart may spend aiming, and how far off it may leave the robot
// before the segment is abandoned instead of driven.
constexpr int kRamseteTurnTimeoutMs = 1500;
constexpr float kRamseteStartHeadingLimitDeg = 10.0;

// Half-width of the heading smoothing and differentiation window, in rows.
// 2 means five rows, or 0.1 s at the planner's 0.02 s dt.
constexpr int kRamseteSmoothHalfWidth = 2;

// One shared decode buffer. Segments are followed strictly one at a time, so
// giving each path its own array would only cost memory.
RamsetePoint ramseteBuffer[kRamseteMaxPoints];
// Scratch copy of the heading column, so smoothing reads unsmoothed neighbours.
float ramseteThetaScratch[kRamseteMaxPoints];

// Wrap an angle to [-pi, pi] so heading error never blows up across +-pi.
float ramsete_normalize_angle(float radians) {
	if (!std::isfinite(radians)) return 0.0; // +-inf would loop here forever
	while (radians > kRamsetePi) radians -= 2.0 * kRamsetePi;
	while (radians < -kRamsetePi) radians += 2.0 * kRamsetePi;
	return radians;
}

// Linearly sample the trajectory at an arbitrary time so the control loop rate
// is decoupled from the export dt. theta is interpolated through the shortest arc.
RamsetePoint ramsete_sample(const RamsetePoint* traj, int count, float time_s) {
	if (time_s <= traj[0].t) return traj[0];
	if (time_s >= traj[count - 1].t) return traj[count - 1];

	for (int i = 1; i < count; i++) {
		if (time_s <= traj[i].t) {
			const RamsetePoint& a = traj[i - 1];
			const RamsetePoint& b = traj[i];
			float span = b.t - a.t;
			float r = span <= 0.0 ? 0.0 : (time_s - a.t) / span;

			RamsetePoint out;
			out.t = time_s;
			out.x = a.x + (b.x - a.x) * r;
			out.y = a.y + (b.y - a.y) * r;
			out.theta = a.theta + ramsete_normalize_angle(b.theta - a.theta) * r;
			out.v = a.v + (b.v - a.v) * r;
			out.omega = a.omega + (b.omega - a.omega) * r;
			return out;
		}
	}

	return traj[count - 1];
}

// Convert a wheel's linear velocity (in/s) into a V5 motor velocity (cartridge
// rpm) for move_velocity, accounting for the external gear ratio. Deliberately
// unclamped: the caller clamps both sides together so saturation cannot change
// the ratio between them, which is what sets the arc the robot actually drives.
float ramsete_wheel_ips_to_motor_rpm(float wheel_ips) {
	float wheel_circumference = kRamsetePi * kDriveWheelDiameterIn;
	float wheel_rpm = (wheel_ips / wheel_circumference) * 60.0;
	return wheel_rpm * (kMotorCartridgeRpm / kDriveWheelRpm);
}

// Straight-line distance between a row and the one before it.
float ramsete_row_distance(const RamsetePoint* traj, int i) {
	const float dx = traj[i].x - traj[i - 1].x;
	const float dy = traj[i].y - traj[i - 1].y;
	return std::sqrt(dx * dx + dy * dy);
}

} // namespace

float ramsete_heading_to_math_rad(float heading_deg) {
	return (kRamsetePi / 2.0) - (heading_deg * kRamsetePi / 180.0);
}

// The inverse, wrapped into the [0, 360) range LemLib reports and accepts.
// Used to seed odom and to aim the robot before a segment starts.
float ramsete_math_rad_to_heading(float math_rad) {
	float heading = 90.0 - (math_rad * 180.0 / kRamsetePi);
	while (heading < 0.0) heading += 360.0;
	while (heading >= 360.0) heading -= 360.0;
	return heading;
}

// Follow a planner-exported Ramsete trajectory. Blocks until the trajectory time
// elapses and the robot has settled onto the final row, then stops the drive.
// Seed odom to the path's first point first (chassis.setPose(...) or a dsr()
// reset) so the start pose matches the path.
void follow_ramsete(const RamsetePoint* traj, int count) {
	if (traj == nullptr || count < 2) return;

	const float total_time = traj[count - 1].t;
	const float settle_heading_rad = kRamseteSettleHeadingDeg * kRamsetePi / 180.0;
	uint32_t start_ms = pros::millis();
	uint32_t loop_ms = start_ms;

	while (true) {
		float t = (pros::millis() - start_ms) / 1000.0;

		RamsetePoint goal = ramsete_sample(traj, count, t);
		const bool settling = t >= total_time;
		// Once the trajectory is over there is nothing left to feed forward, and
		// the final row's own v and omega are whatever the rebuild happened to
		// leave there -- circle_test_1 ends on 0.25 rad/s. Handed to the law as a
		// feedforward that would out-pull the heading correction and quietly spin
		// the robot for the whole settle. Drop both: the goal is now a pose to
		// hold, so only the correction terms should act.
		if (settling) {
			goal.v = 0.0;
			goal.omega = 0.0;
		}

		lemlib::Pose pose = chassis.getPose(); // x, y in inches; theta in degrees
		float theta = ramsete_heading_to_math_rad(pose.theta);

		// Position and heading error in the field frame.
		float error_x = goal.x - pose.x;
		float error_y = goal.y - pose.y;
		float error_theta = ramsete_normalize_angle(goal.theta - theta);

		// Rotate the position error into the robot's local frame.
		float local_x = std::cos(theta) * error_x + std::sin(theta) * error_y;
		float local_y = -std::sin(theta) * error_x + std::cos(theta) * error_y;

		// Update all three errors every iteration. The controller's physical
		// display refresh may be slower than this control loop.
		display_ramsete_errors(local_x, local_y, -error_theta * 180.0 / kRamsetePi);

		// Past the end of the trajectory ramsete_sample pins the goal to the final
		// row, which carries v = 0, so keep running the same law against it until
		// the robot actually arrives. Breaking on elapsed time alone would stop
		// the segment wherever the drivetrain's lag happened to leave it.
		if (settling) {
			const bool arrived = std::fabs(local_x) <= kRamseteSettleAlongTrackIn &&
			                     std::fabs(error_theta) <= settle_heading_rad;
			if (arrived || t >= total_time + kRamseteSettleTimeoutS) break;
		}

		// Ramsete control law. sinc(error_theta) avoids the divide-by-zero as the
		// heading error goes to 0.
		const float gain_v = std::fmax(std::fabs(goal.v), kRamseteMinGainSpeedIps);
		float k = 2.0 * kRamseteZeta * std::sqrt(goal.omega * goal.omega + kRamseteB * gain_v * gain_v);
		float sinc = std::fabs(error_theta) < 1e-6 ? 1.0 : std::sin(error_theta) / error_theta;
		float v_cmd = goal.v * std::cos(error_theta) + k * local_x;
		float omega_cmd = goal.omega + k * error_theta + kRamseteB * goal.v * sinc * local_y;

		// Differential-drive split. omega is counter-clockwise positive, matching
		// the math frame, so the right wheel speeds up on a left (CCW) turn.
		float left_ips = v_cmd - (omega_cmd * kRamseteEffectiveTrackWidthIn / 2.0);
		float right_ips = v_cmd + (omega_cmd * kRamseteEffectiveTrackWidthIn / 2.0);

		float left_rpm = ramsete_wheel_ips_to_motor_rpm(left_ips);
		float right_rpm = ramsete_wheel_ips_to_motor_rpm(right_ips);

		// Scale both sides down together when either one asks for more than the
		// cartridge can give. Clipping each wheel on its own would change the
		// difference between them, which turns a saturated straight into a curve.
		const float peak_rpm = std::fmax(std::fabs(left_rpm), std::fabs(right_rpm));
		if (peak_rpm > kMotorCartridgeRpm) {
			const float scale = kMotorCartridgeRpm / peak_rpm;
			left_rpm *= scale;
			right_rpm *= scale;
		}

		// Round rather than truncate: a cast rounds toward zero, which leaves both
		// wheels commanded systematically slow every single iteration.
		leftMotors.move_velocity(static_cast<std::int32_t>(std::lround(left_rpm)));
		rightMotors.move_velocity(static_cast<std::int32_t>(std::lround(right_rpm)));

		// delay_until holds a true 100 Hz period. Plain delay() adds the loop's own
		// compute time on top of the delay and makes the interval uneven.
		pros::Task::delay_until(&loop_ms, kRamseteLoopMs);
	}

	// brake() applies whatever brake mode the motors are set to, and PROS defaults
	// that to coast -- initialize() sets both drive groups to brake so this
	// actually stops. move_velocity(0) would coast regardless and let the robot
	// drift past the end of the segment before the next one starts.
	leftMotors.brake();
	rightMotors.brake();

	// Show one consistent final endpoint snapshot when the path ends.
	// These remain visible until the next path or driver display replaces them.
	const lemlib::Pose final_pose = chassis.getPose();
	const RamsetePoint& endpoint = traj[count - 1];
	const float final_theta = ramsete_heading_to_math_rad(final_pose.theta);
	const float final_error_x = endpoint.x - final_pose.x;
	const float final_error_y = endpoint.y - final_pose.y;
	const float final_along =
	    std::cos(final_theta) * final_error_x + std::sin(final_theta) * final_error_y;
	const float final_lateral =
	    -std::sin(final_theta) * final_error_x + std::cos(final_theta) * final_error_y;
	const float final_heading_error =
	    -ramsete_normalize_angle(endpoint.theta - final_theta) * 180.0 / kRamsetePi;
	display_ramsete_errors(final_along, final_lateral, final_heading_error, true);
}

int ramsete_parse(const asset& file, RamsetePoint* out, int capacity) {
	const char* cursor = reinterpret_cast<const char*>(file.buf);
	const char* end = cursor + file.size;
	int count = 0;

	while (cursor < end && count < capacity) {
		const char* lineEnd = cursor;
		while (lineEnd < end && *lineEnd != '\n') lineEnd++;

		// Skip comments before copying: the planner appends its whole project as
		// one multi-kilobyte '#' line that would never fit the line buffer.
		if (cursor < lineEnd && *cursor != '#') {
			int length = lineEnd - cursor;
			if (length > kRamseteMaxLineChars - 1) length = kRamseteMaxLineChars - 1;
			char line[kRamseteMaxLineChars];
			std::memcpy(line, cursor, length);
			line[length] = '\0';

			// The asset is not null terminated, so parse the copy rather than the
			// embedded bytes: strtof would otherwise be free to run off the end.
			float values[6];
			int parsed = 0;
			char* field = line;
			while (parsed < 6) {
				char* next = nullptr;
				const float value = std::strtof(field, &next);
				if (next == field) break; // "endData", or a short/ragged row
				values[parsed++] = value;
				field = next;
				while (*field == ',' || *field == ' ' || *field == '\t') field++;
			}

			if (parsed == 6) {
				out[count].t = values[0];
				out[count].x = values[1];
				out[count].y = values[2];
				out[count].theta = values[3];
				out[count].v = values[4];
				out[count].omega = values[5];
				count++;
			}
		}

		cursor = lineEnd + 1;
	}

	return count;
}

// path.jerryio exports its velocity column with no acceleration limit at all.
// These files step v by up to 675 in/s^2, or 17 m/s^2, between consecutive rows;
// a VEX drivetrain does a small fraction of that. Because follow_ramsete indexes
// the trajectory by wall-clock time, the robot falls behind inside the first
// tenth of a second of every segment and the goal point never comes back, so the
// rest of the path gets driven as a chase instead of as a feedforward.
//
// Re-space the timestamps so the same geometric path is asked for at a speed the
// drive can actually reach. x and y are never touched, so the shape of the path
// is exactly what the planner drew; only the speed along it and the time each
// row is due change. On the current exports this costs 2-3% more time.
void ramsete_retime(RamsetePoint* traj, int count) {
	if (count < 2) return;

	// Forward pass: no row may be faster than the one before it plus whatever the
	// drive can gain over the distance between them. Scale the velocity rather
	// than assign it, so a reversing path keeps its sign and a stationary row
	// stays at zero instead of picking up a direction it never had.
	for (int i = 1; i < count; i++) {
		const float ds = ramsete_row_distance(traj, i);
		const float reachable = std::sqrt(traj[i - 1].v * traj[i - 1].v +
		                                  2.0 * kRamseteMaxAccelIps2 * ds);
		const float speed = std::fabs(traj[i].v);
		if (speed > reachable) traj[i].v *= reachable / speed;
	}

	// Backward pass: the same limit read as braking, so the profile arrives at
	// each row already slow enough to make the one after it.
	for (int i = count - 2; i >= 0; i--) {
		const float ds = ramsete_row_distance(traj, i + 1);
		const float reachable = std::sqrt(traj[i + 1].v * traj[i + 1].v +
		                                  2.0 * kRamseteMaxAccelIps2 * ds);
		const float speed = std::fabs(traj[i].v);
		if (speed > reachable) traj[i].v *= reachable / speed;
	}

	// Re-integrate the timestamps from the distances and the new speeds: a row is
	// due once the mean speed across the gap has covered it. Two stationary rows
	// in a row sit on top of each other, so there is no distance to cover and no
	// time to add.
	traj[0].t = 0.0;
	for (int i = 1; i < count; i++) {
		const float ds = ramsete_row_distance(traj, i);
		const float mean = (std::fabs(traj[i - 1].v) + std::fabs(traj[i].v)) / 2.0;
		traj[i].t = traj[i - 1].t + (mean > 1e-4 ? ds / mean : 0.0);
	}
}

// path.jerryio does not export theta and omega as a usable feedforward. It holds
// theta constant across a whole planner segment and then steps it, dumping that
// entire heading change into a single omega sample: these five paths peak at
// 20.7 rad/s, which alone would ask for 109 in/s of wheel difference on a
// drivetrain that tops out near 65. Fed in raw, that is an impulse train, and
// the staircase makes the heading error sawtooth against a robot turning smoothly.
//
// The x and y columns are dense and clean, so rebuild both from the path tangent.
// On the plateaus this reproduces the exported theta to within about a
// thousandth of a radian; between them it fills in the ramp that is missing.
void ramsete_rebuild_heading(RamsetePoint* traj, int count) {
	if (count < 2) return;

	// ramseteBackwards is a per-path setting, so a file is entirely forward or
	// entirely reverse. Read the direction off the fastest row, which is never
	// one of the zero-velocity endpoints where the sign carries no information.
	int fastest = 0;
	for (int i = 1; i < count; i++) {
		if (std::fabs(traj[i].v) > std::fabs(traj[fastest].v)) fastest = i;
	}
	const bool reversed = traj[fastest].v < 0.0;

	// A reversing segment drives back-first, so the robot's heading is the
	// direction of travel turned by pi.
	for (int i = 0; i < count; i++) {
		const int before = i > 0 ? i - 1 : 0;
		const int after = i < count - 1 ? i + 1 : count - 1;
		const float dx = traj[after].x - traj[before].x;
		const float dy = traj[after].y - traj[before].y;
		// A stationary row has no tangent. Keep its exported heading, which is
		// the plateau value and already correct.
		if (std::fabs(dx) < 1e-5 && std::fabs(dy) < 1e-5) continue;
		traj[i].theta = std::atan2(dy, dx) + (reversed ? kRamsetePi : 0.0);
	}

	// Unwrap into one continuous run so interpolating and differentiating theta
	// never sees a jump where the angle crosses +-pi.
	for (int i = 1; i < count; i++) {
		traj[i].theta =
		    traj[i - 1].theta + ramsete_normalize_angle(traj[i].theta - traj[i - 1].theta);
	}

	// Smooth the unwrapped heading before differentiating it. The planner lays a
	// segment down as straight chords between sparse heading updates, so the raw
	// tangent kinks at every join: on part 1 those kinks alone read as 10 rad/s,
	// roughly three times that turn's real rate. A centred average over five rows
	// (0.1 s at the planner's 0.02 s dt) removes them and leaves the genuine turn
	// near 4 rad/s. The window shrinks at the ends, so the headings the segment
	// actually starts and finishes on are preserved to a few hundredths of a degree.
	for (int i = 0; i < count; i++) ramseteThetaScratch[i] = traj[i].theta;
	for (int i = 0; i < count; i++) {
		int reach = kRamseteSmoothHalfWidth;
		if (i < reach) reach = i;
		if (count - 1 - i < reach) reach = count - 1 - i;
		float sum = 0.0;
		for (int j = i - reach; j <= i + reach; j++) sum += ramseteThetaScratch[j];
		traj[i].theta = sum / (2 * reach + 1);
	}

	// Differentiate over the same width. A single-row difference would put the
	// kinks straight back into omega.
	for (int i = 0; i < count; i++) {
		int before = i - kRamseteSmoothHalfWidth;
		int after = i + kRamseteSmoothHalfWidth;
		if (before < 0) before = 0;
		if (after > count - 1) after = count - 1;
		const float span = traj[after].t - traj[before].t;
		traj[i].omega = span > 1e-6 ? (traj[after].theta - traj[before].theta) / span : 0.0;
	}
}

bool follow_ramsete_asset(const asset& file, RamseteStart start) {
	const int count = ramsete_parse(file, ramseteBuffer, kRamseteMaxPoints);
	if (count < 2) return false;
	// A full buffer means the decode stopped at the cap rather than at the end of
	// the file. Refuse it: following a silently truncated path drives the robot
	// part way down it and then stops mid-field.
	if (count >= kRamseteMaxPoints) return false;

	// Re-time before rebuilding the heading, not after. ramsete_rebuild_heading
	// differentiates theta against the timestamps, so it has to see the final ones.
	ramsete_retime(ramseteBuffer, count);
	ramsete_rebuild_heading(ramseteBuffer, count);

	const float startHeading = ramsete_math_rad_to_heading(ramseteBuffer[0].theta);
	if (start == RamseteStart::seedPose) {
		chassis.setPose(ramseteBuffer[0].x, ramseteBuffer[0].y, startHeading);
	} else if (start == RamseteStart::turnToStart) {
		chassis.turnToHeading(startHeading, kRamseteTurnTimeoutMs);
		chassis.waitUntilDone();
		// waitUntilDone returns on the timeout just as readily as on a settled
		// turn, so check where the turn actually left the robot. A few degrees the
		// follower absorbs on its own; a turn that barely moved would mean driving
		// the whole segment sideways, so abandon it instead.
		const float residual = std::fabs(ramsete_normalize_angle(
		    ramsete_heading_to_math_rad(startHeading) -
		    ramsete_heading_to_math_rad(chassis.getPose().theta)));
		if (residual > kRamseteStartHeadingLimitDeg * kRamsetePi / 180.0) return false;
	}

	follow_ramsete(ramseteBuffer, count);
	return true;
}
