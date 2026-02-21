# Lab 2 - Kinematics with UFACTORY Lite6

## Description
This laboratory implements kinematic control for the UFACTORY Lite6 robot using ROS 2. The project includes a custom node capable of calculating trajectories and drawing text using the robotic arm in a simulated environment.

## Project Structure

```
Lab2/
├── src/
│   ├── lite6_drawing/       # Main package for robot control
│   ├── pymoveit2/          # Library for MoveIt2 interface (submodule)
│   └── xarm_ros2/          # Official ROS 2 package for xArm control (submodule)
└── README.md
```


## Dependencies

This project uses the following external repositories as Git submodules:

- **pymoveit2**: [AndrejOrsula/pymoveit2](https://github.com/AndrejOrsula/pymoveit2)
  - Commit: `900c137499ec70d5a5cb216e24d88e91c7a508fc`
  - Python interface for MoveIt2

- **xarm_ros2**: [xArm-Developer/xarm_ros2](https://github.com/xArm-Developer/xarm_ros2)
  - Commit: `5bb832f72ca665f1236a9d8ed1c3a82f308db489`
  - Official ROS 2 support for xArm/Lite6 robots

## Initial Setup

To clone this project for the first time with all its integrated submodules:

`git clone --recursive <repository-url>`

If you have already cloned the repository but the submodule folders are empty, update them with:

`git submodule update --init --recursive`

## Installation and Build

1. Ensure you have ROS 2 (Humble) installed and configured on your system.
2. Navigate to your workspace's source folder and install the package dependencies:
   ```
   cd Lab2/src
   rosdep install --from-paths . --ignore-src -r -y
   ```

3. Return to the root of the workspace and build all packages (including submodules):
  ```
   cd ..
   colcon build
  ```

## Usage

To visualize and execute the drawing routine, you must run the robot simulation and the control node simultaneously. 

Make sure you are at the root of your workspace (`Lab2/`) in both terminals.

**Terminal 1: Start the robot environment (RViz and MoveIt2)**
Source the ROS 2 environment and launch the simulated (fake) Lite 6 arm:

```
source install/setup.bash
ros2 launch xarm_moveit_config lite6_moveit_fake.launch.py
```

**Terminal 2: Run the drawing node**
In a new terminal, source the environment again and run the main service:

```
source install/setup.bash
ros2 run lite6_drawing drawer_node
```

*The script will look for the robot's current position, wait for a brief moment, and then begin generating and executing the Cartesian trajectories to draw.*
