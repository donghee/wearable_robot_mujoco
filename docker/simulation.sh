#!/bin/sh

cd "$(dirname "$0")"

docker compose up -d

./stop.sh simulation

CONTAINER_NAME=wearable_robot_mujoco
SIMULATION_NODE=simulation_node

echo "Starting $SIMULATION_NODE in container $CONTAINER_NAME"
docker exec -it -d $CONTAINER_NAME bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install && source /wearable_ws/install/setup.sh && ros2 run wearable_robot_mujoco $SIMULATION_NODE"
