## ENPM661 Project3 Phase 2 Part 2

Project demonstrating A* algorithm and visualizing the route using non-holonomic constraints and Gazebo

## Dependencies:

* __time__ - tracks computation time

* __turtlebot3__ - simulation setup

* __ROS2+__ - Communication medium for Gazebo and python scripts

* __Gazebo__ - Visualization software for showing turtlebot3 movement

## Gazebo Simulation Instructions:

1. Copy controller file into catkin_ws/src/turtlebot3_teleop/nodes directory
2. Copy turtlebot3_ENPM661.launch into catkin_ws/src/turtlebot3_teleop/nodes directory
3. Copy map.world into catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/worlds directory
3. Launch package by typing roslaunch turtlebot3_gazebo turtlebot3_ENPM661.launch
4. Press ctrl+C to exit 

## Outputs:

* Gazebo visualization window

* No text outputs