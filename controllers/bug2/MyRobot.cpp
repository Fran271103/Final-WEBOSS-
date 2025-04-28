#include "MyRobot.h"
#include <cmath>
#include <webots/Robot.hpp>
#include <iostream>

MyRobot::MyRobot() {
    // Initialize motors
    _left_motor = getMotor("left wheel motor");
    _right_motor = getMotor("right wheel motor");
    _left_motor->setPosition(INFINITY);
    _right_motor->setPosition(INFINITY);
    _left_motor->setVelocity(0.0);
    _right_motor->setVelocity(0.0);

    // Initialize position sensors
    _left_wheel_sensor = getPositionSensor("left wheel sensor");
    _right_wheel_sensor = getPositionSensor("right wheel sensor");
    _left_wheel_sensor->enable(TIME_STEP);
    _right_wheel_sensor->enable(TIME_STEP);

    // Initialize compass
    _my_compass = getCompass("compass");
    _my_compass->enable(TIME_STEP);

    // Initialize GPS
    _my_gps = getGPS("gps");
    _my_gps->enable(TIME_STEP);

    // Initialize distance sensors
    for (int i = 0; i < 16; i++) {
        char buffer[10];
        sprintf(buffer, "ds%d", i);
        _distance_sensor[i] = getDistanceSensor(buffer);
        _distance_sensor[i]->enable(TIME_STEP);
    }

    // Initialize cameras
    _front_cam = getCamera("camera_f");
    _spher_cam = getCamera("camera_s");
    _front_cam->enable(TIME_STEP);
    _spher_cam->enable(TIME_STEP);

    // Initialize odometry
    _sl = 0.0;
    _sr = 0.0;

    // Initialize position and orientation
    _x = 0.0;
    _y = 0.0;
    _theta = 0.0;

    // Set goal
    _x_goal = _path_goal[_active_point][0];
    _y_goal = _path_goal[_active_point][1];
    _theta_goal = atan2(_y_goal - _y, _x_goal - _x);
  
  
    std::cout << "Goals: x: " << _x_goal << ", y: " << _y_goal << ", theta: " << convert_rad_to_deg(_theta_goal) << std::endl;
    
    
    // Initialize mode for Bug2
    _mode = 0; // Start in go-to-goal mode
}

MyRobot::~MyRobot() {
    // Cleanup if necessary
}

void MyRobot::run() {
    int s = 0;
    while (step(TIME_STEP) != -1) {
        this->compute_odometry(true);
        this->print_odometry();
        s += goal_reached();
        std::cout << "goal reached: " << s << std::endl;
        if (goal_reached()) {
            if (_active_point < _total_points - 1) {
                _active_point++;
                _x_goal = _path_goal[_active_point][0];
                _y_goal = _path_goal[_active_point][1];
                _theta_goal = atan2(_y_goal - _y, _x_goal - _x);
            } else {
                stop();
                break;
            }
        }
        go_route();
    }
}

void MyRobot::go_route() {
    if (goal_reached()) {
        // Stop the robot
        _left_motor->setVelocity(0.0);
        _right_motor->setVelocity(0.0);
        return;
    }

    if (_mode == 0) {
        // Check for obstacles
        bool obstacle_ahead = false;
        if (_distance_sensor[14]->getValue() > 0.5 || // Left-front
            _distance_sensor[15]->getValue() > 0.5 || // Front-left
            _distance_sensor[0]->getValue() > 0.5 ||  // Front
            _distance_sensor[1]->getValue() > 0.5 ||  // Front-right
            _distance_sensor[2]->getValue() > 0.5) {  // Right-front
            std::cout << "ds: 14: " << _distance_sensor[14]->getValue() << ", 15: " << _distance_sensor[15]->getValue() << ", 0: " << _distance_sensor[0]->getValue() << ", 1: " << _distance_sensor[1]->getValue() << ", 2: " << _distance_sensor[2]->getValue() << std::endl;
            obstacle_ahead = true;
        }

        if (!obstacle_ahead) {
            // Normal go-to-goal behavior
            float angle_diff = compute_angle_goal();
            if (fabs(angle_diff) > DEGREE_TOLERANCE * M_PI / 180.0) {
                head_goal();
                std::cout << "Align towards goal" << std::endl;
            } else {
                move_forward();
                std::cout << "Forwards" << std::endl;
            }
        } else {
            // Switch to wall-following mode
            _mode = 1;
        }
    } else if (_mode == 1) {
        std::cout << "Wall following" << std::endl;
        // Wall-following mode
        float desired_distance = 0.15; // Desired distance from wall
        float K = 2.0; // Proportional gain
        float forward_speed = MEDIUM_SPEED; // Base speed
        float ds14_value = _distance_sensor[14]->getValue(); // Left-front sensor
        float error = desired_distance - ds14_value;
        float differential_speed = K * error;
        float left_speed = forward_speed + differential_speed;
        float right_speed = forward_speed - differential_speed;

        // Clamp speeds
        if (left_speed > MAX_SPEED) left_speed = MAX_SPEED;
        if (left_speed < -MAX_SPEED) left_speed = -MAX_SPEED;
        if (right_speed > MAX_SPEED) right_speed = MAX_SPEED;
        if (right_speed < -MAX_SPEED) right_speed = -MAX_SPEED;

        // Set motor speeds
        _left_motor->setVelocity(left_speed);
        _right_motor->setVelocity(right_speed);

        // Check if we can switch back to go-to-goal mode
        float ds0_value = _distance_sensor[0]->getValue(); // Front sensor
        float angle_diff = compute_angle_goal();
        if (ds0_value > 0.2 && fabs(angle_diff) < 45.0 * M_PI / 180.0) {
            _mode = 0; // Switch back to go-to-goal mode
        }
    }
}

void MyRobot::compute_odometry(bool use_compass) {
    // Get wheel displacements
    float new_sl = encoder_tics_to_meters(this->_left_wheel_sensor->getValue());
    float new_sr = encoder_tics_to_meters(this->_right_wheel_sensor->getValue());
    float diff_sl = new_sl - _sl;
    float diff_sr = new_sr - _sr;
    _sl = new_sl;
    _sr = new_sr;
    
   
    // Update position
    _x = _x + ((diff_sr + diff_sl) / 2 * cos(_theta + (diff_sr - diff_sl)/(2 * WHEELS_DISTANCE)));
    _y = _y + ((diff_sr + diff_sl) / 2 * sin(_theta + (diff_sr - diff_sl)/(2 * WHEELS_DISTANCE)));

    // Update orientation
    if (use_compass) {
        _theta = convert_bearing_to_radians();
    } else {
        _theta = _theta + ((diff_sr - diff_sl)/WHEELS_DISTANCE);
        // Wrap theta to [-pi, pi]
        while (_theta > M_PI) _theta -= 2*M_PI;
        while (_theta < -M_PI) _theta += 2*M_PI;
    }
}

double MyRobot::convert_bearing_to_radians() {
    const double *north = _my_compass->getValues();
    double rad = atan2(north[0], north[2]);
    return rad;
}

double MyRobot::convert_bearing_to_degrees() {
    return convert_rad_to_deg(convert_bearing_to_radians());
}

double MyRobot::convert_deg_to_rad(double deg) {
    return deg * M_PI / 180.0;
}

double MyRobot::convert_rad_to_deg(double rad) {
    return rad * 180.0 / M_PI;
}

float MyRobot::encoder_tics_to_meters(float tics) {
    return tics * WHEEL_RADIUS;
}

void MyRobot::print_odometry() {
    std::cout << "Position: (" << _x << ", " << _y << "), Orientation: " << convert_rad_to_deg(_theta) << " degrees" << std::endl;
}

bool MyRobot::goal_reached() {
    return compute_distance_goal() < DISTANCE_TOLERANCE;
}

float MyRobot::compute_distance_goal() {
    return sqrt(pow(_x_goal - _x, 2) + pow(_y_goal - _y, 2));
}

float MyRobot::compute_angle_goal() {
    double desired_theta = atan2(_y_goal - _y, _x_goal - _x);
    double angle_diff = desired_theta - _theta;
    while (angle_diff > M_PI) angle_diff -= 2*M_PI;
    while (angle_diff < -M_PI) angle_diff += 2*M_PI;
    return angle_diff;
}

void MyRobot::head_goal() {
    float angle_diff = compute_angle_goal();
    float speed = angle_diff * 10.0; // Proportional control
    _left_motor->setVelocity(-speed);
    _right_motor->setVelocity(speed);
}

void MyRobot::move_forward() {
    float distance = compute_distance_goal();
    float speed = std::min(MEDIUM_SPEED, distance * 5.0); // Proportional speed
    _left_motor->setVelocity(speed);
    _right_motor->setVelocity(speed);
}

void MyRobot::stop() {
    _left_motor->setVelocity(0.0);
    _right_motor->setVelocity(0.0);
}