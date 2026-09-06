#include "22020Vlib/dsr.hpp"

#include "robot/hardware.hpp"
#include <cmath>

float dsr_deg_to_rad(float degrees) {
	return degrees * 3.1415926535 / 180.0;
}

float dsr_normalize_heading(float heading) {
	while (heading < 0) heading += 360;
	while (heading >= 360) heading -= 360;
	return heading;
}

DsrSensorConfig dsr_get_sensor_config(RobotDsrSensor sensor) {
	switch (sensor) {
		case RobotDsrSensor::front:
			return dsrFrontSensor;
		case RobotDsrSensor::right:
			return dsrRightSensor;
		case RobotDsrSensor::back:
			return dsrBackSensor;
		case RobotDsrSensor::left:
			return dsrLeftSensor;
	}

	return {0, 0, 0};
}

float dsr_read_sensor_inches(RobotDsrSensor sensor) {
	// TODO: return the selected physical distance sensor's reading in inches.
	(void)sensor;
	return -1;
}

float dsr_wall_angle(DsrWall wall) {
	switch (wall) {
		case DsrWall::top:
			return 0;
		case DsrWall::right:
			return 90;
		case DsrWall::bottom:
			return 180;
		case DsrWall::left:
			return 270;
	}

	return 0;
}

DsrWall dsr_nearest_wall_for_sensor(RobotDsrSensor sensor, float heading) {
	DsrSensorConfig config = dsr_get_sensor_config(sensor);
	float sensor_heading = dsr_normalize_heading(heading + config.heading_offset);

	if (sensor_heading >= 315 || sensor_heading < 45) {
		return DsrWall::top;
	} else if (sensor_heading < 135) {
		return DsrWall::right;
	} else if (sensor_heading < 225) {
		return DsrWall::bottom;
	}

	return DsrWall::left;
}

float dsr_tracking_point_distance_to_wall(RobotDsrSensor sensor, DsrWall wall, float heading) {
	float sensor_distance = dsr_read_sensor_inches(sensor);
	if (sensor_distance < dsrMinValidDistance || sensor_distance > dsrMaxValidDistance) {
		return -1;
	}

	DsrSensorConfig config = dsr_get_sensor_config(sensor);
	float theta = dsr_deg_to_rad((heading + config.heading_offset) - dsr_wall_angle(wall));
	float blue = std::cos(theta) * sensor_distance;
	float purple = std::cos(theta) * config.x_offset;
	float green = std::sin(theta) * config.y_offset * -1;

	return blue + purple + green;
}

namespace {

void dsr_add_coordinate_sample(float& x_sum, int& x_count, float& y_sum, int& y_count,
                               RobotDsrSensor sensor, DsrWall wall, float heading) {
	float distance_to_wall = dsr_tracking_point_distance_to_wall(sensor, wall, heading);
	if (distance_to_wall < 0) {
		return;
	}

	switch (wall) {
		case DsrWall::top:
			y_sum += dsrTopWallY - distance_to_wall;
			y_count++;
			break;
		case DsrWall::bottom:
			y_sum += dsrBottomWallY + distance_to_wall;
			y_count++;
			break;
		case DsrWall::right:
			x_sum += dsrRightWallX - distance_to_wall;
			x_count++;
			break;
		case DsrWall::left:
			x_sum += dsrLeftWallX + distance_to_wall;
			x_count++;
			break;
	}
}

void dsr_add_sensor_coordinate_sample(float& x_sum, int& x_count, float& y_sum, int& y_count,
                                      RobotDsrSensor sensor, float heading) {
	DsrWall wall = dsr_nearest_wall_for_sensor(sensor, heading);
	dsr_add_coordinate_sample(x_sum, x_count, y_sum, y_count, sensor, wall, heading);
}

} // namespace

void dsr(bool use_front_sensor, bool use_right_sensor, bool use_back_sensor, bool use_left_sensor) {
	lemlib::Pose pose = chassis.getPose();
	float heading = dsr_normalize_heading(pose.theta);

	float x_sum = 0;
	float y_sum = 0;
	int x_count = 0;
	int y_count = 0;

	if (use_front_sensor) {
		dsr_add_sensor_coordinate_sample(x_sum, x_count, y_sum, y_count, RobotDsrSensor::front, heading);
	}
	if (use_right_sensor) {
		dsr_add_sensor_coordinate_sample(x_sum, x_count, y_sum, y_count, RobotDsrSensor::right, heading);
	}
	if (use_back_sensor) {
		dsr_add_sensor_coordinate_sample(x_sum, x_count, y_sum, y_count, RobotDsrSensor::back, heading);
	}
	if (use_left_sensor) {
		dsr_add_sensor_coordinate_sample(x_sum, x_count, y_sum, y_count, RobotDsrSensor::left, heading);
	}

	float new_x = x_count > 0 ? x_sum / x_count : pose.x;
	float new_y = y_count > 0 ? y_sum / y_count : pose.y;
	if (x_count > 0 || y_count > 0) {
		chassis.setPose(new_x, new_y, pose.theta);
	}
}
