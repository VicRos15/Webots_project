#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/GPS.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/PositionSensor.hpp>

#include <iostream>
#include <cmath>
#include <vector>
#include <limits>


using namespace webots;

// Enumeración de estados del robot
enum State {
    ORIENTING,
    MOVING_FORWARD,
    AVOIDING_OBSTACLE,
    ARRIVED
};

// Constantes de control
const double TARGET_X = 9.0;
const double TARGET_Y = 12.0;
const double LINEAR_SPEED = 6.0;
const double TURN_SPEED = 1.0;
const double ANGLE_TOLERANCE = 0.1;
const double POSITION_TOLERANCE = 0.5;
const double OBSTACLE_THRESHOLD = 500.0;
const double SAFE_DISTANCE = 700.0; 
const double WHEEL_RADIUS = 0.0205; // Radio de las ruedas
const double WHEEL_BASE = 0.058; // Distancia entre ruedas 

// Variables globales

Motor* motor_left;
Motor* motor_right;
GPS* gps;
InertialUnit* imu;
DistanceSensor* ds_front_l;
DistanceSensor* ds_front_r;
DistanceSensor* ds_left;
DistanceSensor* ds_right;
PositionSensor* encoder_left;
PositionSensor* encoder_right;

std::vector<std::pair<double, double>> trajectory;

State current_state = ORIENTING;
double prev_left_encoder = 0.0, prev_right_encoder = 0.0;
double odom_x = 1.5, odom_y = 1.5, odom_theta = 0.0;

// Funciones de movimiento
void move_forward() {
    motor_left->setVelocity(LINEAR_SPEED);
    motor_right->setVelocity(LINEAR_SPEED);
}

void turn_left() {
    motor_left->setVelocity(-TURN_SPEED);
    motor_right->setVelocity(TURN_SPEED);
}

void turn_right() {
    motor_left->setVelocity(TURN_SPEED);
    motor_right->setVelocity(-TURN_SPEED);
}

void stop() {
    motor_left->setVelocity(0.0);
    motor_right->setVelocity(0.0);
}

// Cálculo de ángulo hacia el objetivo
double calculate_target_angle(double x, double y) {
    return atan2(TARGET_Y - y, TARGET_X - x);
}

void update_odometry() {
    double left_enc = encoder_left->getValue();
    double right_enc = encoder_right->getValue();
    
    double d_left = (left_enc - prev_left_encoder) * WHEEL_RADIUS;
    double d_right = (right_enc - prev_right_encoder) * WHEEL_RADIUS;
    double d_center = (d_left + d_right) / 2.0;
    double d_theta = (d_right - d_left) / WHEEL_BASE;
    
    odom_x += d_center * cos(odom_theta);
    odom_y += d_center * sin(odom_theta);
    odom_theta += d_theta;
    
    prev_left_encoder = left_enc;
    prev_right_encoder = right_enc;
    
    trajectory.push_back({odom_x, odom_y});
}

void run_trajectory() {
    update_odometry();
    
    const double* pos = gps->getValues();
    const double* imu_rads = imu->getRollPitchYaw();
    double distance_front_l = ds_front_l->getValue();
    double distance_front_r = ds_front_r->getValue();
    double distance_left = ds_left->getValue();
    double distance_right = ds_right->getValue();
    
    double robot_x = pos[0];
    double robot_y = pos[1];
    double yaw = imu_rads[2];
    
    double target_angle = calculate_target_angle(robot_x, robot_y);
    double angle_error = target_angle - yaw;
    
    // Normalizar ángulo entre -PI y PI
    while (angle_error > M_PI) angle_error -= 2 * M_PI;
    while (angle_error < -M_PI) angle_error += 2 * M_PI;

    if (std::hypot(TARGET_X - robot_x, TARGET_Y - robot_y) < POSITION_TOLERANCE) {
        current_state = ARRIVED;
    }
    std::cout << "GPS: x = " << robot_x << ", y = " << robot_y << std::endl;
    std::cout << "IMU (Yaw): " << yaw << " rad" << std::endl;
    std::cout << "Odometría: x = " << odom_x << ", y = " << odom_y << ", θ = " << odom_theta << " rad" << std::endl;
    std::cout << "Sensores de distancia: Front L = " << distance_front_l 
            << ", Front R = " << distance_front_r 
            << ", Left = " << distance_left 
            << ", Right = " << distance_right << std::endl;
    std::cout << "--------------------------------------" << std::endl;


    switch (current_state) {
        case ORIENTING:
            if (std::abs(angle_error) > ANGLE_TOLERANCE) {
                (angle_error > 0) ? turn_left() : turn_right();
            } else {
                current_state = MOVING_FORWARD;
            }
            break;
        
        case MOVING_FORWARD:
            if (distance_front_l < OBSTACLE_THRESHOLD || distance_front_r < OBSTACLE_THRESHOLD) {
                stop();
                current_state = AVOIDING_OBSTACLE;
            } else {
                move_forward();
            }
            break;
        
        case AVOIDING_OBSTACLE:
            if (distance_left > distance_right) {
                turn_left();
            } else {
                turn_right();
            }
            if (distance_front_l >= OBSTACLE_THRESHOLD && distance_front_r >= OBSTACLE_THRESHOLD) {
                move_forward();
            }
            if (distance_left >= SAFE_DISTANCE && distance_right >= SAFE_DISTANCE) {
                current_state = ORIENTING;
            }
            break;
        
        case ARRIVED:
            stop();
            std::cout << "¡Objetivo alcanzado!" << std::endl;
            break;
    }
}

int main(int argc, char **argv) {
    Robot *robot = new Robot();
    int timeStep = (int)robot->getBasicTimeStep();

    motor_left = robot->getMotor("left wheel motor");
    motor_right = robot->getMotor("right wheel motor");
    gps = robot->getGPS("gps");
    imu = robot->getInertialUnit("inertial unit");
    ds_front_l = robot->getDistanceSensor("front_ds_l");
    ds_front_r = robot->getDistanceSensor("front_ds_r");
    ds_left = robot->getDistanceSensor("left_ds");
    ds_right = robot->getDistanceSensor("right_ds");
    encoder_left = robot->getPositionSensor("left wheel sensor");
    encoder_right = robot->getPositionSensor("right wheel sensor");

    gps->enable(timeStep);
    imu->enable(timeStep);
    ds_front_l->enable(timeStep);
    ds_front_r->enable(timeStep);
    ds_left->enable(timeStep);
    ds_right->enable(timeStep);
    encoder_left->enable(timeStep);
    encoder_right->enable(timeStep);

    motor_left->setPosition(std::numeric_limits<double>::infinity());
    motor_right->setPosition(std::numeric_limits<double>::infinity());

    
    while (robot->step(timeStep) != -1) {
        run_trajectory();
    }
    
    delete robot;
    return 0;
}
