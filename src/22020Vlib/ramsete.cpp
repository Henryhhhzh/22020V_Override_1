#include "22020Vlib/ramsete.hpp"

#include "robot/config.hpp"
#include "robot/hardware.hpp"
#include "pros/rtos.hpp"
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>

namespace {

// Ramsete b has units rad^2 / distance^2. Convert the usual 2.0 rad^2/m^2
// baseline to inches because trajectory velocities and pose errors use inches.
// Using 2.0 directly here makes position and heading corrections far too strong.
constexpr float kMetersPerInch = 0.0254;
constexpr float kRamseteB = 2.0 * kMetersPerInch * kMetersPerInch;
constexpr float kRamseteZeta = 0.7;

constexpr float kRamsetePi = 3.14159265358979;
constexpr int kRamseteLoopMs = 10; // 100 Hz control loop
constexpr int kRamseteMaxLineChars = 128;

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

// Follow a planner-exported Ramsete trajectory. Blocks until the trajectory
// time elapses, then stops the drive. Seed odom to the path's first point first
// (chassis.setPose(...) or a dsr() reset) so the start pose matches the path.
void follow_ramsete(const RamsetePoint* traj, int count) {
	if (traj == nullptr || count < 2) return;

	uint32_t start_ms = pros::millis();

	while (true) {
		float t = (pros::millis() - start_ms) / 1000.0;
		if (t >= traj[count - 1].t) break;

		RamsetePoint goal = ramsete_sample(traj, count, t);
		lemlib::Pose pose = chassis.getPose(); // x, y in inches; theta in degrees
		float theta = ramsete_heading_to_math_rad(pose.theta);

		// Position and heading error in the field frame.
		float error_x = goal.x - pose.x;
		float error_y = goal.y - pose.y;
		float error_theta = ramsete_normalize_angle(goal.theta - theta);

		// Rotate the position error into the robot's local frame.
		float local_x = std::cos(theta) * error_x + std::sin(theta) * error_y;
		float local_y = -std::sin(theta) * error_x + std::cos(theta) * error_y;

		// Ramsete control law. sinc(error_theta) avoids the divide-by-zero as the
		// heading error goes to 0.
		float k = 2.0 * kRamseteZeta * std::sqrt(goal.omega * goal.omega + kRamseteB * goal.v * goal.v);
		float sinc = std::fabs(error_theta) < 1e-6 ? 1.0 : std::sin(error_theta) / error_theta;
		float v_cmd = goal.v * std::cos(error_theta) + k * local_x;
		float omega_cmd = goal.omega + k * error_theta + kRamseteB * goal.v * sinc * local_y;

		// Differential-drive split. omega is counter-clockwise positive, matching
		// the math frame, so the right wheel speeds up on a left (CCW) turn.
		float left_ips = v_cmd - (omega_cmd * kTrackWidthIn / 2.0);
		float right_ips = v_cmd + (omega_cmd * kTrackWidthIn / 2.0);

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

		leftMotors.move_velocity(static_cast<int>(left_rpm));
		rightMotors.move_velocity(static_cast<int>(right_rpm));

		pros::delay(kRamseteLoopMs);
	}

	// brake() holds position; move_velocity(0) would coast to a stop and let the
	// robot drift past the end of the segment before the next one starts.
	leftMotors.brake();
	rightMotors.brake();
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
	ramsete_rebuild_heading(ramseteBuffer, count);

	const float startHeading = ramsete_math_rad_to_heading(ramseteBuffer[0].theta);
	if (start == RamseteStart::seedPose) {
		chassis.setPose(ramseteBuffer[0].x, ramseteBuffer[0].y, startHeading);
	} else if (start == RamseteStart::turnToStart) {
		chassis.turnToHeading(startHeading, 1500);
		chassis.waitUntilDone();
	}

	follow_ramsete(ramseteBuffer, count);
	return true;
}
