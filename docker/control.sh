#!/bin/sh

cd "$(dirname "$0")"

CONTAINER_NAME=wearable_robot_mujoco

docker exec -it $CONTAINER_NAME bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install && source /wearable_ws/install/setup.sh && ros2 run wearable_robot_mujoco elbow_vel_cmd_node"
