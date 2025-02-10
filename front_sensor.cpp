#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <iostream>
#include <limits>

using namespace webots;

const double LINEAR_SPEED = 3.0;
const double STOP_DISTANCE = 0.5;  // Distancia para detenerse (ajusta según el sensor)

int main() {
    Robot *robot = new Robot();
    int timeStep = (int)robot->getBasicTimeStep();

    // Inicializar motores
    Motor *motor_left = robot->getMotor("left wheel motor");
    Motor *motor_right = robot->getMotor("right wheel motor");

    // Inicializar sensor de distancia
    DistanceSensor *ds_front = robot->getDistanceSensor("front_ds");
    ds_front->enable(timeStep);

    // Configurar los motores para movimiento infinito
    motor_left->setPosition(std::numeric_limits<double>::infinity());
    motor_right->setPosition(std::numeric_limits<double>::infinity());

    while (robot->step(timeStep) != -1) {
        double distance = ds_front->getValue();
        std::cout << "Front sensor distance: " << distance << std::endl;

        if (distance < STOP_DISTANCE) {
            std::cout << "Obstacle detected! Stopping." << std::endl;
            motor_left->setVelocity(0.0);
            motor_right->setVelocity(0.0);
        } else {
            motor_left->setVelocity(LINEAR_SPEED);
            motor_right->setVelocity(LINEAR_SPEED);
        }
    }

    delete robot;
    return 0;
}
