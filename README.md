# turtle_control_ros2

A ROS 2 package for controlling turtlesim turtles with different movement patterns. This package provides educational examples for learning ROS 2 concepts including publishers, subscribers, services, and control algorithms.

## Overview

This package contains two main nodes for controlling turtles in the ROS 2 turtlesim simulator:

1. **turtle_circle_node**: A template node for making a turtle drive in a circle (to be completed as an exercise)
2. **turtle_move_goal_node**: A complete implementation that moves a turtle to a goal position using PI control

## Prerequisites

- ROS 2 (tested with Humble, but should work with other distributions)
- Python 3
- turtlesim package

## Installation

1. Navigate to your ROS 2 workspace source directory:
```bash
cd ~/ros2_ws/src
```

2. Clone this repository:
```bash
git clone https://github.com/christianpfitzner/turtle_control_ros2.git
```

3. Build the package:
```bash
cd ~/ros2_ws
colcon build --packages-select turtle_control
```

4. Source the workspace:
```bash
source ~/ros2_ws/install/setup.bash
```

## Usage

### Running the Circle Node (Tutorial Exercise)

This node is a template for learning ROS 2. The goal is to make the turtle drive in a circle by implementing the `timer_callback` method.

1. Start the turtlesim node:
```bash
ros2 run turtlesim turtlesim_node
```

2. In a new terminal, run the circle node:
```bash
ros2 run turtle_control turtle_circle_node
```

**Exercise**: Complete the `timer_callback` method in `turtle_circle_node.py` to publish Twist messages that make the turtle move in a circle.

### Running the Goal-Based Movement Node

This node demonstrates a complete implementation of goal-based turtle navigation using PI control.

1. Start the turtlesim node:
```bash
ros2 run turtlesim turtlesim_node
```

2. In a new terminal, run the goal node:
```bash
ros2 run turtle_control turtle_move_goal_node
```

3. Publish a goal position to move the turtle:
```bash
ros2 topic pub /turtle1/goal turtlesim/msg/Pose "{x: 5.0, y: 5.0, theta: 0.0}"
```

The turtle will automatically navigate to the specified position.

## Package Structure

```
turtle_control_ros2/
├── turtle_control/
│   ├── __init__.py
│   ├── turtle_circle_node.py      # Template for circle movement
│   └── turtle_move_goal_node.py   # Goal-based navigation with PI control
├── test/
│   ├── test_copyright.py
│   ├── test_flake8.py
│   └── test_pep257.py
├── package.xml                     # Package manifest
├── setup.py                        # Python package setup
└── README.md                       # This file
```

## Nodes

### turtle_circle_node

**Subscribed Topics:**
- `my_turtle/pose` (turtlesim/msg/Pose): Current pose of the turtle

**Published Topics:**
- `my_turtle/cmd_vel` (geometry_msgs/msg/Twist): Velocity commands for the turtle

**Services:**
- Uses `spawn` and `kill` services from turtlesim

### turtle_move_goal_node

**Subscribed Topics:**
- `turtle1/pose` (turtlesim/msg/Pose): Current pose of the turtle
- `turtle1/goal` (turtlesim/msg/Pose): Desired goal position

**Published Topics:**
- `turtle1/cmd_vel` (geometry_msgs/msg/Twist): Velocity commands for the turtle

**Control Algorithm:**
- Linear velocity: P controller based on distance to goal
- Angular velocity: PI controller based on heading error with anti-windup

## Learning Objectives

This package is designed to teach:
- Creating ROS 2 Python nodes
- Publishing and subscribing to topics
- Using service clients
- Implementing basic control algorithms (P and PI controllers)
- Working with geometry_msgs and turtlesim messages

## Testing

Run the package tests:
```bash
cd ~/ros2_ws
colcon test --packages-select turtle_control
```

## Maintainer

**Prof. Dr. Christian Pfitzner**
- Email: christian.pfitzner@th-nuernberg.de 

## License

TODO: License declaration

## Contributing

This is an educational package. Feel free to fork and modify for your own learning purposes.
