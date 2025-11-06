#!/bin/sh

cd "$(dirname "$0")"

CONTAINER_NAME=wearable_robot_mujoco

echo $@
docker compose up -d
docker exec -it $CONTAINER_NAME bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install && source /wearable_ws/install/setup.sh && ros2 run wearable_robot_mujoco simulation_node"
