// File:          my_controller.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/GPS.hpp>
#include <webots/InertialUnit.hpp>

#include <iostream>
#include <limits>
#include <cmath>

// All the webots classes are defined in the "webots" namespace
using namespace webots;

// Trayectory states
enum State{
    MOVING_X,
    MOVING_Y,
    TURNING_LEFT,
    TURNING_RIGHT,
    ARRIVED
};

// Constantes destino
constexpr double target_x = 8.5;
constexpr double target_y = 12.5;
constexpr double lineal_speed = 6.28;
constexpr double turn_speed = 1.0;
constexpr double angle_tol = 1.0;
constexpr double position_tol = 0.1;

// Variables globales
Motor* motor_left;
Motor* motor_right;
State current_state = MOVING_X;

// Functions

//Movement
void move_forward(){
    motor_left->setVelocity(lineal_speed);
    motor_right->setVelocity(lineal_speed);
}

void move_backward(){
    motor_left->setVelocity(-lineal_speed);
    motor_right->setVelocity(-lineal_speed);
}

void turn_left(){
    motor_left->setVelocity(-turn_speed);
    motor_right->setVelocity(turn_speed);
}

void turn_right(){
    motor_left->setVelocity(turn_speed);
    motor_right->setVelocity(-turn_speed);
}

void stop() {
    motor_left->setVelocity(0.0);
    motor_right->setVelocity(0.0);
}

// Función de trayectoria optimizada
void run_trajectory(const double* pos, const double* imu_degrees) {
    const double yaw = imu_degrees[2];  // Ángulo en grados

    switch (current_state) {
        case MOVING_X:
            if (pos[0] >= 0 && pos[0] <= 3.5 + position_tol){
                move_forward();
            }
            else if ( pos[1]>8 - position_tol && pos[1]<9 + position_tol && pos[0]< 8.5 + position_tol){
                move_forward();
            }
            else {
                stop();
                current_state = TURNING_LEFT;
            }
            break;

        case TURNING_LEFT:
            if (std::abs(yaw - 90.0) > angle_tol) {
                turn_left();
            } else {
                stop();
                current_state = MOVING_Y;
            }
            break;
        case TURNING_RIGHT:
            if (std::abs(yaw + 0.0) > angle_tol) {
                turn_right();
            } else {
                stop();
                current_state = MOVING_X;
            }
            break;

        case MOVING_Y:
            if (pos[1] >= 0 && pos[1] <= 8.5 + position_tol) {
                move_forward();
            } 
            else if (pos[0] > 8 - position_tol && pos[0] < 9 + position_tol && pos[1] < 12.5 + position_tol) {
                move_forward();
            }
            else {
                stop();
                current_state = TURNING_RIGHT;
            }
            break;

        case ARRIVED:
            stop();
            std::cout << "¡Objetivo alcanzado!" << std::endl;
            break;
    }
}

// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();

  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();

  // You should insert a getDevice-like function in order to get the
  // instance of a device of the robot. Something like:
  motor_left = robot->getMotor("left wheel motor");
  motor_right = robot->getMotor("right wheel motor");
  // DistanceSensor *ds = robot->getDistanceSensor("dsname");
  webots::GPS* gps = robot->getGPS("gps");
  webots::InertialUnit* imu = robot->getInertialUnit("inertial unit");

  //// ds->enable(timeStep);
  gps->enable(timeStep);
  imu->enable(timeStep);

  motor_left->setVelocity(0.0);
  motor_right->setVelocity(0.0);
  
  motor_left->setPosition(std::numeric_limits<double>::infinity());
  motor_right->setPosition(std::numeric_limits<double>::infinity());

  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(timeStep) != -1) {
    // Read the sensors:
    // Enter here functions to read sensor data, like:
    // double val = ds->getValue();
    const double * pos = gps->getValues();
    const double * imu_rads = imu->getRollPitchYaw();
       std::cout << "GPS: ["
                  << pos[0] << " "
                  << pos[1] << " "
                  << pos[2] << "] IMU: ["
                  << imu_rads[0]*180.0/3.14159 << " "
                  << imu_rads[1]*180.0/3.14159 << " "
                  << imu_rads[2]*180.0/3.14159 << "]" << std::endl;

    // Process sensor data here.
    const double imu_deg[3] = {
        imu_rads[0] * 180.0 / 3.14159, // Roll in degrees
        imu_rads[1] * 180.0 / 3.14159, // Pitch in degrees
        imu_rads[2] * 180.0 / 3.14159  // Yaw in degrees
    };    
    run_trajectory(pos, imu_deg);

  };

  // Enter here exit cleanup code.
  std::cout << "Bye from c++!" << std::endl;

  delete robot;
  return 0;
}
 