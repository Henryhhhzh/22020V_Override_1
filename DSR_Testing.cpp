#include <cmath>
#include <iomanip>
#include <iostream>

enum class SensorSide {
    front,
    right,
    back,
    left
};

enum class WallSide {
    top,
    right,
    bottom,
    left
};

struct SensorReadings {
    float front;
    float right;
    float back;
    float left;
};

struct SensorConfig {
    float heading_offset;
    float x_offset;
    float y_offset;
};

constexpr float TOP_WALL_Y = 72.0;
constexpr float RIGHT_WALL_X = 72.0;
constexpr float BOTTOM_WALL_Y = -72.0;
constexpr float LEFT_WALL_X = -72.0;
constexpr float MIN_VALID_DISTANCE = 0.5;
constexpr float MAX_VALID_DISTANCE = 100.0;

float deg_to_rad(float degrees) {
    return degrees * 3.1415926535 / 180.0;
}

float normalize_heading(float heading) {
    while (heading < 0) heading += 360;
    while (heading >= 360) heading -= 360;
    return heading;
}

SensorConfig sensor_config(SensorSide sensor) {
    switch (sensor) {
        case SensorSide::front:
            return {0, 0, 0};
        case SensorSide::right:
            return {90, 0, 0};
        case SensorSide::back:
            return {180, 0, 0};
        case SensorSide::left:
            return {270, 0, 0};
    }

    return {0, 0, 0};
}

float reading_for_sensor(SensorSide sensor, SensorReadings readings) {
    switch (sensor) {
        case SensorSide::front:
            return readings.front;
        case SensorSide::right:
            return readings.right;
        case SensorSide::back:
            return readings.back;
        case SensorSide::left:
            return readings.left;
    }

    return -1;
}

float wall_angle(WallSide wall) {
    switch (wall) {
        case WallSide::top:
            return 0;
        case WallSide::right:
            return 90;
        case WallSide::bottom:
            return 180;
        case WallSide::left:
            return 270;
    }

    return 0;
}

const char* sensor_name(SensorSide sensor) {
    switch (sensor) {
        case SensorSide::front:
            return "robot front";
        case SensorSide::right:
            return "robot right";
        case SensorSide::back:
            return "robot back";
        case SensorSide::left:
            return "robot left";
    }

    return "unknown";
}

WallSide nearest_wall_for_sensor(SensorSide sensor, float heading) {
    SensorConfig config = sensor_config(sensor);
    float sensor_heading = normalize_heading(heading + config.heading_offset);

    if (sensor_heading >= 315 || sensor_heading < 45) {
        return WallSide::top;
    } else if (sensor_heading < 135) {
        return WallSide::right;
    } else if (sensor_heading < 225) {
        return WallSide::bottom;
    }

    return WallSide::left;
}

float distance_from_tracking_point_to_wall(SensorSide sensor, WallSide wall, float heading,
                                           SensorReadings readings) {
    float sensor_distance = reading_for_sensor(sensor, readings);
    if (sensor_distance < MIN_VALID_DISTANCE || sensor_distance > MAX_VALID_DISTANCE) {
        return -1;
    }

    SensorConfig config = sensor_config(sensor);
    float theta = deg_to_rad((heading + config.heading_offset) - wall_angle(wall));
    float blue = std::cos(theta) * sensor_distance;
    float purple = std::cos(theta) * config.x_offset;
    float green = std::sin(theta) * config.y_offset * -1;

    return blue + purple + green;
}

void add_coordinate_sample(float& x_sum, int& x_count, float& y_sum, int& y_count,
                           SensorSide sensor, WallSide wall, float heading, SensorReadings readings) {
    float distance_to_wall = distance_from_tracking_point_to_wall(sensor, wall, heading, readings);
    if (distance_to_wall < 0) {
        std::cout << sensor_name(sensor) << " reading ignored\n";
        return;
    }

    switch (wall) {
        case WallSide::top:
            y_sum += TOP_WALL_Y - distance_to_wall;
            y_count++;
            std::cout << sensor_name(sensor) << " -> top wall gives Y = "
                      << TOP_WALL_Y - distance_to_wall << "\n";
            break;
        case WallSide::bottom:
            y_sum += BOTTOM_WALL_Y + distance_to_wall;
            y_count++;
            std::cout << sensor_name(sensor) << " -> bottom wall gives Y = "
                      << BOTTOM_WALL_Y + distance_to_wall << "\n";
            break;
        case WallSide::right:
            x_sum += RIGHT_WALL_X - distance_to_wall;
            x_count++;
            std::cout << sensor_name(sensor) << " -> right wall gives X = "
                      << RIGHT_WALL_X - distance_to_wall << "\n";
            break;
        case WallSide::left:
            x_sum += LEFT_WALL_X + distance_to_wall;
            x_count++;
            std::cout << sensor_name(sensor) << " -> left wall gives X = "
                      << LEFT_WALL_X + distance_to_wall << "\n";
            break;
    }
}

void add_sensor_coordinate_sample(float& x_sum, int& x_count, float& y_sum, int& y_count,
                                  SensorSide sensor, float heading, SensorReadings readings) {
    WallSide wall = nearest_wall_for_sensor(sensor, heading);
    add_coordinate_sample(x_sum, x_count, y_sum, y_count, sensor, wall, heading, readings);
}

void calculate_and_print_coords(float heading, SensorReadings readings) {
    heading = normalize_heading(heading);

    std::cout << std::fixed << std::setprecision(2);
    std::cout << "Heading: " << heading << "\n";
    std::cout << "Using physical front/right/back/left sensors directly\n";

    float x_sum = 0;
    float y_sum = 0;
    int x_count = 0;
    int y_count = 0;

    add_sensor_coordinate_sample(x_sum, x_count, y_sum, y_count, SensorSide::front, heading, readings);
    add_sensor_coordinate_sample(x_sum, x_count, y_sum, y_count, SensorSide::right, heading, readings);
    add_sensor_coordinate_sample(x_sum, x_count, y_sum, y_count, SensorSide::back, heading, readings);
    add_sensor_coordinate_sample(x_sum, x_count, y_sum, y_count, SensorSide::left, heading, readings);

    if (x_count > 0) {
        std::cout << "Final X: " << x_sum / x_count << "\n";
    } else {
        std::cout << "Final X: unavailable\n";
    }

    if (y_count > 0) {
        std::cout << "Final Y: " << y_sum / y_count << "\n";
    } else {
        std::cout << "Final Y: unavailable\n";
    }
}

int main() {
    float heading = 0;
    SensorReadings readings {};

    std::cout << "Enter heading degrees only, example: 0\n> ";
    if (!(std::cin >> heading)) {
        std::cout << "Invalid heading. Type only the number, not the prompt text.\n";
        return 1;
    }

    std::cout << "Enter sensor readings in inches as: front right back left\n";
    std::cout << "Example: 24 18 40 30\n> ";
    if (!(std::cin >> readings.front >> readings.right >> readings.back >> readings.left)) {
        std::cout << "Invalid readings. Type four numbers only, separated by spaces.\n";
        return 1;
    }

    calculate_and_print_coords(heading, readings);
}
