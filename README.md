Overview
This package provides a single node ( draw_rectangle ) that subscribes to /turtle1/pose and publishes velocity commands to /turtle1/cmd_vel. Rather than relying on open-loop timer counts, the node reads the turtle's actual position and heading at each control step to decide when to stop moving and when to stop turning. This eliminates the angular drift that causes tick-based approaches to produce skewed parallelograms instead of true rectangles.

turtlesim_rectangle/
├── draw_rectangle.py   # main node — pose-feedback rectangle driver
├── package.xml         # ROS 2 package metadata and dependencies
├── setup.cfg           # entry point configuration
├── setup.py            # ament_python build configuration
└── README.md


Dependencies
rclpy -ROS 2 Python client library
geometry_msgs - Twist message for velocity commands
turtlesimPose - message + simulator node



Usage


Terminal 1 — start the simulator:
ros2 run turtlesim turtlesim_node

Terminal 2 — run the rectangle node:

Clone or copy the package into your workspace
cp -r turtlesim_rectangle ~/ros2_ws/src/

Build
cd ~/ros2_ws
colcon build --packages-select turtlesim_rectangle

Source
source install/setup.bash

Run
ros2 run turtlesim_rectangle draw_rectangle


Custom dimensions
Parameters can be passed at runtime:

ros2 run turtlesim_rectangle draw_rectangle --ros-args \

  -p width:=4.0 \
  
  -p height:=2.5 \
  
  -p speed:=2.0 \
  
  -p repeats:=3

