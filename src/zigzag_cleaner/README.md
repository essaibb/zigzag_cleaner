Zigzag Cleaner Robot (ROS)
 Project Overview

This project implements an autonomous  robot system using ROS (Robot Operating System) for indoor mapping and area coverage.
The robot operates in two main phases:

Wall-Following Mapping:
The robot follows walls to build a map of the environment using SLAM.

Zigzag Coverage Cleaning:
After mapping, the robot performs a systematic vertical zigzag motion to cover the area, similar to a robotic vacuum cleaner.

The project is designed and tested in Gazebo simulation using a TurtleBot3-based platform.

Project Objectives

Perform autonomous navigation in an indoor environment

Build a 2D occupancy grid map using SLAM

Apply wall-following behavior with safe distance control

Execute structured zigzag area coverage

Avoid obstacles such as walls and table legs

Stop automatically after completing a predefined number of rows

Organize the project using ROS best practices (nodes, launch files, package structure)

Robot Platform

Robot: TurtleBot3 (Waffle recommended for better stability and sensing)

Sensors:

2D LiDAR (LaserScan)

Odometry

Simulation: Gazebo

Visualization: RViz 

System Architecture

1. Wall Following Mapping Node

File: wall_follow_node.py

Responsibilities:

Follows the right wall at a fixed distance

Detects front obstacles

Performs 90° left turns when encountering walls

Provides stable motion for SLAM mapping

Key Topics Used:

/scan (LaserScan)

/odom (Odometry)

/cmd_vel (Twist)

2. Zigzag Coverage Node

File: zigzag_node.py

Responsibilities:

Moves in vertical zigzag rows

Detects walls using LaserScan

Shifts between rows with controlled lateral motion

Applies a localized increase in shift distance to bypass table legs

Stops automatically after completing 8 rows

Key Features:

State machine–based control

Row counter for task completion

Controlled shutdown at the end of coverage

zigzag_cleaner/
├── launch/
│   ├── wall_following.launch
│   ├── zigzag_node.launch
│   
│
├── scripts/
│   ├── wall_follow_node.py
│   └── zigzag_node.py
│
├── maps/
       ├── room_map.pgm
│      └── room_map.yaml

├── package.xml
├── CMakeLists.txt
└── README.md





 How to Run the Project
 Build the Workspace
cd ~/catkin_ws
catkin_make
source devel/setup.bash

2. Wall Following + SLAM Mapping
roslaunch zigzag_cleaner wall_following.launch


This will:

Launch Gazebo

Start SLAM

Run the wall-following node

3. Save the Map

After mapping is complete:

rosrun map_server map_saver -f room_map


This generates:

room_map.pgm

room_map.yaml


4. Zigzag Coverage Cleaning
roslaunch zigzag_cleaner zigzag_node.launch

The robot will:

Perform vertical zigzag coverage

Stop automatically after row 8

Tested Environment

Ubuntu 20.04

ROS Noetic

Gazebo 11

TurtleBot3 Simulation Packages

 Notes

The system uses modular ROS nodes

Navigation behavior is implemented using finite state machines

Obstacle handling is achieved through sensor-based logic and localized path adaptation

The project follows ROS software engineering best practices

Explanation Summary 

The robot first maps the environment using wall-following SLAM. Afterward, it performs systematic zigzag coverage to ensure full area traversal. The system stops automatically after completing a predefined number of rows.

 Author

Name: ESSA NOMAN SAAD
Project: Zigzag Cleaner Robot
Course:  Robotics / ROS

 License

This project is released under the BSD License.


