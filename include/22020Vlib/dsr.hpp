#pragma once

#include "robot/config.hpp"

// Distance Sensor Reset. During autonomous, odometry drifts: wheels slip, the
// robot gets pushed, and encoder distance stops matching reality. DSR reads the
// field walls with distance sensors and corrects the position estimate.
//
// It corrects x and y only. Heading is left exactly as the IMU reports it,
// because the IMU is the one odom input that does not drift from wheel slip.

// Reset odom from any combination of physical distance sensors.
// Argument order: front, right, back, left.
//
// Each enabled sensor picks the wall it is currently pointing at, converts its
// reading into a field coordinate, and contributes one sample. Samples are
// averaged per axis, so an axis no sensor could see keeps its current value.
// A sensor whose reading falls outside dsrMinValidDistance..dsrMaxValidDistance
// is discarded, so if every sensor is out of range the pose is left untouched.
void dsr(bool use_front_sensor, bool use_right_sensor, bool use_back_sensor, bool use_left_sensor);

// The steps dsr() is built from. Exposed for testing individual sensors and for
// calibrating the mounting offsets in robot/config.hpp.

float dsr_deg_to_rad(float degrees);
float dsr_normalize_heading(float heading);

DsrSensorConfig dsr_get_sensor_config(RobotDsrSensor sensor);

// Reading from one physical sensor, in inches. Returns -1 when unavailable.
float dsr_read_sensor_inches(RobotDsrSensor sensor);

float dsr_wall_angle(DsrWall wall);

// Which wall a sensor faces, given the robot's heading and that sensor's
// mounting angle.
DsrWall dsr_nearest_wall_for_sensor(RobotDsrSensor sensor, float heading);

// Perpendicular distance from the tracking center to a wall, correcting for
// where the sensor sits relative to that center. Returns -1 on a bad reading.
float dsr_tracking_point_distance_to_wall(RobotDsrSensor sensor, DsrWall wall, float heading);
