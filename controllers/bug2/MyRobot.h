#ifndef MY_ROBOT_H
#define MY_ROBOT_H

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Compass.hpp>
#include <webots/GPS.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Camera.hpp>

#define TIME_STEP 64
#define MAX_SPEED 10.0
#define MEDIUM_SPEED 5.0
#define DEGREE_TOLERANCE 5
#define DISTANCE_TOLERANCE 0.1
#define WHEELS_DISTANCE 0.3606
#define WHEEL_RADIUS 0.0825
#define ENCODER_TICS_PER_RADIAN 1

class MyRobot : public webots::Robot {
public:
    MyRobot();
    ~MyRobot();
    void run();

private:
    // Motors
    webots::Motor *_left_motor;
    webots::Motor *_right_motor;

    // Position Sensors
    webots::PositionSensor *_left_wheel_sensor;
    webots::PositionSensor *_right_wheel_sensor;

    // Compass
    webots::Compass *_my_compass;

    // GPS
    webots::GPS *_my_gps;

    // Distance Sensors
    webots::DistanceSensor *_distance_sensor[16];
    const char *ds_name[16] = {"ds0", "ds1", "ds2", "ds3", "ds4", "ds5", "ds6", "ds7", "ds4", "ds9", "ds10", "ds11", "ds12", "ds13", "ds14", "ds15"};

    // Cameras
    webots::Camera *_front_cam;
    webots::Camera *_spher_cam;

    // Path Goal
    float _path_goal[1][2] = {{0, -17}}; // Single goal at (16, 0)
    int _active_point = 0;
    const int _total_points = 1;

    // Current position and orientation
    float _x, _y, _theta;

    // Goal position
    float _x_goal, _y_goal, _theta_goal;

    // Mode for Bug2 algorithm
    int _mode; // 0: go-to-goal, 1: wall-following

    // Odometry
    float _sl, _sr; // Cumulative wheel distances

    // Functions
    void compute_odometry(bool use_compass = false);
    double convert_bearing_to_degrees();
    double convert_bearing_to_radians();
    double convert_deg_to_rad(double deg);
    double convert_rad_to_deg(double rad);
    float encoder_tics_to_meters(float tics);
    void print_odometry();
    bool goal_reached();
    void go_route();
    float compute_distance_goal();
    float compute_angle_goal();
    void head_goal();
    void move_forward();
    void stop();
};

#endif // MY_ROBOT_H