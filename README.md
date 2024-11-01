# Fuzzy Logic Controller for TurtleBot3

This repository implements a fuzzy logic controller for the TurtleBot3 robot in ROS2, supporting both **Control Architecture** and **Subsumption Architecture** approaches. It enables the TurtleBot3 to perform tasks like **right-edge following** and **obstacle avoidance** using fuzzy logic and layered behavior systems.

## Control and Subsumption Architectures

This implementation includes two main architectures:

1. **Control Architecture**: A fuzzy logic-based control system that processes sensor inputs to manage TurtleBot3’s behaviors. The controller uses membership functions and fuzzy rules to assess distances and adjust speed and rotation for **right-edge following** and **obstacle avoidance**.

2. **Subsumption Architecture**: A layered behavior-based control structure where behaviors are organized hierarchically. Each layer represents a behavior, allowing higher-priority layers (like obstacle avoidance) to override or subsume lower-priority layers (like edge following) when specific conditions are met. The architecture ensures that the robot responds to immediate threats while maintaining its primary navigation goals.

3. **Fuzzy Logic Integration**: Uses fuzzy logic principles to enable TurtleBot3 to make decisions based on sensor inputs, offering flexibility in complex environments.

## Setup

### Dependencies
- ROS2 Foxy
- Python libraries: `rclpy`, `numpy`

### Simulation Environment
For running the code in a Gazebo simulator:
```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

For running code on a robot:
```
ros2 launch turtlebot3_bringup robot.launch.py
python3 main.py
```

