# Project Webots

This project features a custom CSV map generated based on my name, **Victor Rosillo Suero**. The system integrates multiple sensors and provides a robust API for data processing and visualization. Additionally, the GUI has been enhanced with texture customization for the map display.

## Custom map

The result of creating a map using my surname is this map in csv:

![CSV map](images/map_csv.png)


This is the original map, created using the script **webots-map-from-csv_R2022b.py**

![Original map](images/map_orig.png)

Using the new script I have implemented to create a map with textures

![Textures map](images/map_with_textures.png)




## Implemented Sensors

The system includes the following sensors:

- **GPS Sensor**: Used to obtain precise location data.
- **IMU (Inertial Measurement Unit) Sensor**: Provides orientation, acceleration, and angular velocity data.
- **Position Sensor (Encoder)**: Used to obtain the odometry.
- **Distance Sensor**: Used to avoid obstacles.

## API

The project includes these functionalities:

### API webots extras
- **Position Sensor (Encoder)**: Implement a position controller on each wheel to update the odometry. Moreover, it's useful to compare the odometry with the GPS output for redundancy. (7.5%)
- **Distance Snesor**: Implement 4 laser distance sensor, two in the front, and one on each side. The controller use them to change the system state.(7.5%) 

### Constant Variables

The following constants define the robot's control parameters:

- **TARGET_X** - X coordinate of the target destination.
- **TARGET_Y** - Y coordinate of the target destination.
- **LINEAR_SPEED** - Maximum linear speed of the robot (6.0).
- **TURN_SPEED** - Speed at which the robot rotates (1.0).
- **ANGLE_TOLERANCE** - Allowed angular deviation (0.1).
- **POSITION_TOLERANCE** - Allowed positional deviation (0.5).
- **OBSTACLE_THRESHOLD** - Sensor reading threshold to detect obstacles (500.0).
- **SAFE_DISTANCE** - Minimum safe distance to obstacles (700.0).
- **WHEEL_RADIUS** - Radius of the wheels (0.0205).
- **WHEEL_BASE** - Distance between the wheels (0.058).

### Control
- `run_trajectory()` - Changes the state to perform the optimal trajectory.
- `calculate_target_angle(x, y)` - Computes the angle required to reach the target.
- `update_odometry()` - Updates the robot's position using encoder data.


### Movement
- `move_forward()` - Moves the robot forward at a constant speed.
- `move_backward()` - Moves the robot backward at a constant speed.
- `turn_left()` - Rotates the robot to the left.
- `turn_right()` - Rotates the robot to the right.
- `stop()` - Stops the robot's movement.

### Sensor Functions
- `getGPS()` - Retrieves real-time GPS coordinates.
- `getInertialUnit()` - Fetches IMU sensor readings.
- `getMotor()` - Create motor object for each wheel.
- `setVelocity()` - Allows changing the motor velocity dynamically.
- `setPosition()` - Set the motor position.

### Trajectory States
The robot follows a state-based trajectory system:

- `ORIENTING` - Adjusts its orientation to align with the target direction before moving.
- `MOVING_FORWARD` - Moves straight toward the target while avoiding minor obstacles.
- `AVOIDING_OBSTACLE` - Executes a maneuver to bypass an obstacle detected by the sensors.
- `ARRIVED` - Stops moving upon reaching the target location.

## GUI extra
Significant improvements have been made to the GUI, including:

- **Map Texture Customization**: Users can now change the textures of the map for better visualization and user experience. You could change textures including new images. To create the new map with textures you have to run **map_from_scv_textures.py** script. (2.5%)

## Notes

- [Upgrade from R2023a to R2023b](https://cyberbotics.com/doc/guide/from-2023a-to-2023b): No relevant changes
- [Upgrade from R2022a to R2022b](https://cyberbotics.com/doc/guide/from-2022a-to-2022b): EXTERNPROTO
- [Upgrade from R2021b to R2022a](https://cyberbotics.com/doc/guide/from-2021b-to-2022a): Changes in orientation

