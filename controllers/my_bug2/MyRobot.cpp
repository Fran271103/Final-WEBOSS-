#include "MyRobot.h"
#include <cmath>
#include <webots/Robot.hpp>
#include <iostream>

MyRobot::MyRobot() {
    _time_step = 64;
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

    //initialize distance sensors
    for (int ind = 0; ind < NUM_DISTANCE_SENSOR; ind++){
        cout << "Initializing distance sensor: " << ds_name[ind] << endl;
        _distance_sensor[ind] = getDistanceSensor(ds_name[ind]);
        _distance_sensor[ind]->enable(_time_step);
    } 

    // Initialize cameras
    _front_cam = getCamera("camera_f");
    _spher_cam = getCamera("camera_s");
    _front_cam->enable(TIME_STEP);
    _spher_cam->enable(TIME_STEP);

    // Initialize odometry
    _sl = 0; // If using encoder_tics_to_radians doesn't work -> nan
    _sr = 0;

    // Initialize position and orientation
    _x = 0.0;
    _y = 0.0;
    _theta = 0.0;

    // Set goal
    _x_goal = _path_goal[_active_point][0];
    _y_goal = _path_goal[_active_point][1];
    _theta_goal = atan2(_y_goal - _y, _x_goal - _x);

    // Initialize mode for Bug2
    _mode = 0; // Start in go-to-goal mode
}

MyRobot::~MyRobot() {
    // Stop motors
    _left_motor->setVelocity(0.0);
    _right_motor->setVelocity(0.0);
    
    // Disable robot's sensors
    _my_compass->disable();
    _left_wheel_sensor->disable();
    _right_wheel_sensor->disable();
    _my_gps->disable();
    _front_cam->disable();
    _spher_cam->disable();
}

void MyRobot::run() {
    while (step(TIME_STEP) != -1) {
        compute_odometry(true);
        print_odometry();
        
        switch(_mode) {
            case 0:
                head_to_goal();
                if (obstacle_detected()) {
                    _mode = 1;
                    _hit_distance = compute_distance_goal();
                }
                break;
            case 1:
                follow_wall();
                if (on_goal_line() && closer_than_hit()) {
                    _mode = 0;
                }
                break;
        }
        cout << "Mode: " << _mode << endl;
        cout << "On line: " << on_goal_line() << endl << "Closer than hit: " << closer_than_hit() << endl;
    }
}

float MyRobot::angle_to_goal() {
    float dx = _x_goal - _x;
    float dy = _y_goal - _y;
    
    float angle_diff = atan2(dy, dx);
    return normalizeAngleRad(angle_diff);
}

void MyRobot::head_to_goal() {
    float angle_goal = angle_to_goal();

    float heading_error = normalizeAngleRad(angle_goal - _theta);
    
    float gain = min(1.0f, std::abs(heading_error) / static_cast<float>(M_PI));  // range 0–1
    float turn_speed = SLOW_SPEED * gain;
    
    cout << _x << " t: " << _y << " dif: " << heading_error <<endl;
    cout << "";
    
    if (abs(heading_error) > 0.3) {
        if (heading_error < 0) {
            turn_left(turn_speed);
        } else {
            turn_right(turn_speed);
        }
    }
    else {
        move_forward(MEDIUM_SPEED);
    }
}

void MyRobot::follow_wall() {
    float d_front = 0.5*(_distance_sensor[0]->getValue()+_distance_sensor[15]->getValue());
    float d_fr = _distance_sensor[13]->getValue();
    float d_right = 0.5*(_distance_sensor[11]->getValue()+_distance_sensor[12]->getValue());
    float d_br = _distance_sensor[10]->getValue();
    
    std::cout << "front: " << d_front << " FR: " << d_fr << " R: " << d_right << std::endl;
    
    // Case 1: Obstacle directly ahead → turn left
    if (d_front > 500) {
        turn_left(SLOW_SPEED);
    }
    // Case 2: Too far from wall → turn right to find it
    else if (d_right < 500 && d_fr < 500){
        turn_right(SLOW_SPEED);
    }
    // Case 2: Too far from wall → turn right to find it
    else if (d_fr < WALL_DISTANCE - 300) {
        turn_right(SLOW_SPEED);
    }
    // Case 3: Too close to wall → turn left slightly
    else if (d_fr > WALL_DISTANCE + 300) {
        turn_left(SLOW_SPEED);
    }
    // Case 4: Wall detected at desired distance → move forward
    else {
        move_forward(MEDIUM_SPEED);
    }
    
}

bool MyRobot::on_goal_line() {
    float angle_goal = angle_to_goal();
    
    cout << "Theta goal: " << _theta_goal << endl;
    
    cout << "Angle to goal: " << angle_goal << endl;
    
    return (abs(angle_goal - _theta_goal) < 0.04);
}

bool MyRobot::closer_than_hit() {
    float distance = compute_distance_goal();
    
    return (distance + 0.1 < _hit_distance);
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

float MyRobot::convert_bearing_to_radians() {
    const double *north = _my_compass->getValues();
    float rad = atan2(north[0], north[2]);
    return rad;
}

float MyRobot::convert_bearing_to_degrees() {
    return convert_rad_to_deg(convert_bearing_to_radians());
}

float MyRobot::convert_deg_to_rad(float deg) {
    return deg * M_PI / 180.0;
}

float MyRobot::convert_rad_to_deg(float rad) {
    return rad * 180.0 / M_PI;
}

float MyRobot::normalizeAngleRad(float angle) {
    while (angle > M_PI) angle -= 2*M_PI;
    while (angle <= -M_PI) angle += 2*M_PI;
    return angle;
}

float MyRobot::encoder_tics_to_meters(float tics) {
    return tics * WHEEL_RADIUS;
}

void MyRobot::print_odometry() {
    cout << "Position: (" << _x << ", " << _y << "), Orientation: " << convert_rad_to_deg(_theta) << " degrees" << endl;
}

bool MyRobot::goal_reached() {
    return (_y <= -17);
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

void MyRobot::move_forward(float speed) {
    _left_motor->setVelocity(speed);
    _right_motor->setVelocity(speed);
}

void MyRobot::turn_left(float speed) {
    _left_motor->setVelocity(-speed);
    _right_motor->setVelocity(speed);
}

void MyRobot::turn_right(float speed) {
    _left_motor->setVelocity(speed);
    _right_motor->setVelocity(-speed);
}

// bool MyRobot::valid_and_close(float distance) {
    // return (distance > SENSOR_MIN_VALID && distance < OBSTACLE_THRESHOLD);
// }

bool MyRobot::obstacle_detected() {
    float d_frontleft = _distance_sensor[14]->getValue();
    float d_front = 0.5*(_distance_sensor[0]->getValue()+_distance_sensor[15]->getValue());
    float d_frontright = _distance_sensor[1]->getValue();
    
    cout << "Front: " << d_front << endl << "Left: " << d_frontleft << endl << "Right: " << d_frontright << endl;

    return (
      (d_frontleft > OBSTACLE_THRESHOLD) ||
      (d_front > OBSTACLE_THRESHOLD) ||
      (d_frontright > OBSTACLE_THRESHOLD)
    );
}

void MyRobot::stop() {
    _left_motor->setVelocity(0.0);
    _right_motor->setVelocity(0.0);
}