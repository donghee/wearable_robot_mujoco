#!/bin/sh

cd "$(dirname "$0")"

./stop.sh controller

# copy task module to patient
patient_name=$(cat ../../Index.dat| head -n 1)
cp ../wearable_robot_mujoco/task.py ../../Patient/$patient_name/task.py

CONTAINER_NAME=wearable_robot_mujoco
CONTOLLER_NODE=elbow_vel_cmd_node

echo "Starting $CONTOLLER_NODE in container $CONTAINER_NAME"
docker exec -it $CONTAINER_NAME bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install && source /wearable_ws/install/setup.sh && ros2 run wearable_robot_mujoco $CONTOLLER_NODE"

