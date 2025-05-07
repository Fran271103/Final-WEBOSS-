#ifndef MY_ROBOT_H
#define MY_ROBOT_H

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Compass.hpp>
#include <webots/GPS.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Camera.hpp>

using namespace std;
using namespace webots;

#define NUM_DISTANCE_SENSOR 16
#define TIME_STEP 64
#define MAX_SPEED 10.0
#define MEDIUM_SPEED 7
#define SLOW_SPEED   2
#define DEGREE_TOLERANCE 5
#define HEADING_TOLERANCE 0.01
#define OBSTACLE_THRESHOLD 300
#define WALL_DISTANCE 900
#define SENSOR_MIN_VALID 0.01
#define WHEELS_DISTANCE 0.3606
#define WHEEL_RADIUS 0.0825
#define ENCODER_TICS_PER_RADIAN 1

using namespace webots;

class MyRobot : public Robot {
public:
    MyRobot();
    ~MyRobot();
    void run();

private:
    int _time_step;
    // Motors
    Motor *_left_motor;
    Motor *_right_motor;

    // Position Sensors
    PositionSensor *_left_wheel_sensor;
    PositionSensor *_right_wheel_sensor;

    // Compass
    Compass *_my_compass;

    // GPS
    GPS *_my_gps;

    // Distance Sensors
    DistanceSensor *_distance_sensor[16];
    const char *ds_name[16] = {"ds0", "ds1", "ds2", "ds3", "ds4", "ds5", "ds6", "ds7", "ds4", "ds9", "ds10", "ds11", "ds12", "ds13", "ds14", "ds15"};

    // Cameras
    Camera *_front_cam;
    Camera *_spher_cam;

    // Path Goal
    float _path_goal[1][2] = {{0, -18}}; // Single goal at (16, 0)
    int _active_point = 0;
    const int _total_points = 1;

    // Current position and orientation
    float _x, _y, _theta;

    // Goal position
    float _x_goal, _y_goal, _theta_goal;

    // Current position and orientation
    float _hit_distance;

    // Mode for Bug2 algorithm
    int _mode;

    // Odometry
    float _sl, _sr; // Cumulative wheel distances

    // Functions
    void compute_odometry(bool use_compass = false);
    float convert_bearing_to_degrees();
    float convert_bearing_to_radians();
    float convert_deg_to_rad(float deg);
    float convert_rad_to_deg(float rad);
    float encoder_tics_to_meters(float tics);
    void print_odometry();
    bool goal_reached();
    void go_route();
    float compute_distance_goal();
    float compute_angle_goal();
    void head_to_goal();
    void move_forward(float speed);
    void turn_left(float speed);
    void turn_right(float speed);
    void stop();
    void to_goal();
    float normalizeAngleRad(float angle);
    bool obstacle_detected();
    bool valid_and_close(float distance);
    float angle_to_goal();
    void follow_wall();
    bool on_goal_line();
    bool closer_than_hit();
};

#endif // MY_ROBOT_H