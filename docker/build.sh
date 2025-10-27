#!/bin/sh

cd /wearable_ws/
colcon build --symlink-install 

#source install/setup.bash
#ros2 run wearable_robot_mujoco simulation_node
#ros2 run wearable_robot_mujoco elbow_vel_cmd_node
