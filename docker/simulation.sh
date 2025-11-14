#!/bin/sh

cd "$(dirname "$0")"

# copy xml from Index.dat to simulation folder
# TODO: use shell script instead of backend API
curl -X POST http://localhost:5000/api/xml/copy

docker compose up -d --force-recreate

./stop.sh simulation

CONTAINER_NAME=wearable_robot_mujoco
SIMULATION_NODE=simulation_node

echo "Starting $SIMULATION_NODE in container $CONTAINER_NAME"
docker exec -it $CONTAINER_NAME bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install && source /wearable_ws/install/setup.sh && ros2 run wearable_robot_mujoco $SIMULATION_NODE"
