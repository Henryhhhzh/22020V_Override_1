#include <cmath>
#include <iomanip>
#include <iostream>

enum class SensorSide {
    top,
    right,
    bottom,
    left
};

enum class WallSide {
    top,
    right,
    bottom,
    left
};

struct SensorReadings {
    float top;
    float right;
    float bottom;
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
        case SensorSide::top:
            return {0, 0, 0};
        case SensorSide::right:
            return {90, 0, 0};
        case SensorSide::bottom:
            return {180, 0, 0};
        case SensorSide::left:
            return {270, 0, 0};
    }

    return {0, 0, 0};
}

float reading_for_sensor(SensorSide sensor, SensorReadings readings) {
    switch (sensor) {
        case SensorSide::top:
            return readings.top;
        case SensorSide::right:
            return readings.right;
        case SensorSide::bottom:
            return readings.bottom;
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
        case SensorSide::top:
            return "robot top";
        case SensorSide::right:
            return "robot right";
        case SensorSide::bottom:
            return "robot bottom";
        case SensorSide::left:
            return "robot left";
    }

    return "unknown";
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

void calculate_and_print_coords(float heading, SensorReadings readings) {
    heading = normalize_heading(heading);

    SensorSide top_facing_sensor = SensorSide::top;
    SensorSide right_facing_sensor = SensorSide::right;
    SensorSide bottom_facing_sensor = SensorSide::bottom;
    SensorSide left_facing_sensor = SensorSide::left;

    if (heading >= 315.5 || heading < 45.5) {
        top_facing_sensor = SensorSide::top;
        right_facing_sensor = SensorSide::right;
        bottom_facing_sensor = SensorSide::bottom;
        left_facing_sensor = SensorSide::left;
    } else if (heading < 135) {
        top_facing_sensor = SensorSide::left;
        right_facing_sensor = SensorSide::top;
        bottom_facing_sensor = SensorSide::right;
        left_facing_sensor = SensorSide::bottom;
    } else if (heading < 225) {
        top_facing_sensor = SensorSide::bottom;
        right_facing_sensor = SensorSide::left;
        bottom_facing_sensor = SensorSide::top;
        left_facing_sensor = SensorSide::right;
    } else {
        top_facing_sensor = SensorSide::right;
        right_facing_sensor = SensorSide::bottom;
        bottom_facing_sensor = SensorSide::left;
        left_facing_sensor = SensorSide::top;
    }

    std::cout << std::fixed << std::setprecision(2);
    std::cout << "Heading: " << heading << "\n";
    std::cout << "Top-facing sensor: " << sensor_name(top_facing_sensor) << "\n";
    std::cout << "Right-facing sensor: " << sensor_name(right_facing_sensor) << "\n";
    std::cout << "Bottom-facing sensor: " << sensor_name(bottom_facing_sensor) << "\n";
    std::cout << "Left-facing sensor: " << sensor_name(left_facing_sensor) << "\n";

    float x_sum = 0;
    float y_sum = 0;
    int x_count = 0;
    int y_count = 0;

    add_coordinate_sample(x_sum, x_count, y_sum, y_count, top_facing_sensor, WallSide::top,
                          heading, readings);
    add_coordinate_sample(x_sum, x_count, y_sum, y_count, bottom_facing_sensor, WallSide::bottom,
                          heading, readings);
    add_coordinate_sample(x_sum, x_count, y_sum, y_count, left_facing_sensor, WallSide::left,
                          heading, readings);
    add_coordinate_sample(x_sum, x_count, y_sum, y_count, right_facing_sensor, WallSide::right,
                          heading, readings);

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

    std::cout << "Enter sensor readings in inches as: top right bottom left\n";
    std::cout << "Example: 24 18 40 30\n> ";
    if (!(std::cin >> readings.top >> readings.right >> readings.bottom >> readings.left)) {
        std::cout << "Invalid readings. Type four numbers only, separated by spaces.\n";
        return 1;
    }

    calculate_and_print_coords(heading, readings);
}
