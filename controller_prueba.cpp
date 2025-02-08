#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/GPS.hpp>
#include <webots/InertialUnit.hpp>

#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <fstream>
#include <sstream>

using namespace webots;

const int MAP_ROWS = 10;
const int MAP_COLS = 14;
int grid[MAP_ROWS][MAP_COLS];

// Cargar el mapa desde un archivo CSV
void loadMap(const std::string& filename) {
    std::ifstream file(filename);
    if (!file) {
        std::cerr << "Error al abrir el archivo " << filename << std::endl;
        return;
    }
    std::string line;
    int row = 0;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        int col = 0, value;
        while (ss >> value) {
            grid[row][col] = value;
            if (ss.peek() == ',') ss.ignore();
            col++;
        }
        row++;
    }
    file.close();
}

// Algoritmo A*
struct Node {
    int x, y;
    double cost, priority;
    Node* parent;
    Node(int x, int y, double cost, double priority, Node* parent = nullptr)
        : x(x), y(y), cost(cost), priority(priority), parent(parent) {}
};

double heuristic(int x1, int y1, int x2, int y2) {
    return std::abs(x1 - x2) + std::abs(y1 - y2);
}

std::vector<std::pair<int, int>> a_star(std::pair<int, int> start, std::pair<int, int> goal) {
    std::priority_queue<Node*, std::vector<Node*>, [](Node* a, Node* b) {
        return a->priority > b->priority;
    }> open_set;

    std::vector<std::vector<bool>> visited(MAP_ROWS, std::vector<bool>(MAP_COLS, false));
    std::vector<std::pair<int, int>> path;

    open_set.push(new Node(start.first, start.second, 0, heuristic(start.first, start.second, goal.first, goal.second)));

    while (!open_set.empty()) {
        Node* current = open_set.top();
        open_set.pop();

        if (current->x == goal.first && current->y == goal.second) {
            while (current) {
                path.emplace_back(current->x, current->y);
                current = current->parent;
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        visited[current->x][current->y] = true;

        for (const auto& dir : std::vector<std::pair<int, int>>{{-1, 0}, {1, 0}, {0, -1}, {0, 1}}) {
            int nx = current->x + dir.first;
            int ny = current->y + dir.second;

            if (nx >= 0 && nx < MAP_ROWS && ny >= 0 && ny < MAP_COLS && grid[nx][ny] == 0 && !visited[nx][ny]) {
                double new_cost = current->cost + 1;
                double priority = new_cost + heuristic(nx, ny, goal.first, goal.second);
                open_set.push(new Node(nx, ny, new_cost, priority, current));
            }
        }
    }
    return {};  // No hay ruta encontrada
}


void move_forward(Motor* motor_left, Motor* motor_right) {
    motor_left->setVelocity(5.0);
    motor_right->setVelocity(5.0);
}

void turn_left(Motor* motor_left, Motor* motor_right) {
    motor_left->setVelocity(-2.0);
    motor_right->setVelocity(2.0);
}

void turn_right(Motor* motor_left, Motor* motor_right) {
    motor_left->setVelocity(2.0);
    motor_right->setVelocity(-2.0);
}

void follow_path(std::vector<std::pair<int, int>> path, Robot* robot, Motor* motor_left, Motor* motor_right, GPS* gps, InertialUnit* imu) {
    for (size_t i = 1; i < path.size(); ++i) {
        std::pair<int, int> prev = path[i - 1];
        std::pair<int, int> current = path[i];

        const double* pos = gps->getValues();
        const double* imu_rads = imu->getRollPitchYaw();
        double current_angle = imu_rads[2] * 180.0 / 3.14159; // Convertir a grados

        if (current.second > prev.second) {
            // Mover a la derecha
            if (current_angle != 90) turn_right(motor_left, motor_right);
        } else if (current.second < prev.second) {
            // Mover a la izquierda
            if (current_angle != -90) turn_left(motor_left, motor_right);
        } else if (current.first > prev.first) {
            // Mover hacia abajo
            if (current_angle != 180) turn_right(motor_left, motor_right);
        } else {
            // Mover hacia arriba
            if (current_angle != 0) turn_left(motor_left, motor_right);
        }

        move_forward(motor_left, motor_right);
        robot->step(1000);
    }
}


int main() {
    Robot* robot = new Robot();
    Motor* motor_left = robot->getMotor("left wheel motor");
    Motor* motor_right = robot->getMotor("right wheel motor");
    GPS* gps = robot->getGPS("gps");
    InertialUnit* imu = robot->getInertialUnit("inertial unit");

    gps->enable(100);
    imu->enable(100);
    
    motor_left->setPosition(std::numeric_limits<double>::infinity());
    motor_right->setPosition(std::numeric_limits<double>::infinity());

    loadMap("map.csv");  // Cargar el mapa desde CSV

    std::pair<int, int> start = {0, 0};
    std::pair<int, int> goal = {9, 12};

    std::vector<std::pair<int, int>> path = a_star(start, goal);
    if (!path.empty()) {
        follow_path(path, robot, motor_left, motor_right, gps, imu);
    } else {
        std::cout << "No se encontró un camino" << std::endl;
    }

    delete robot;
    return 0;
}

