#!/bin/sh

cd "$(dirname "$0")"

./stop.sh controller

# Copy task module to patient
patient_name=$(cat ../../Index.dat| head -n 1)
cp ../wearable_robot_mujoco/task.py ../../Patient/$patient_name/task.py
cp ../wearable_robot_mujoco/wearable_robot_api.py ../../Patient/$patient_name/wearable_robot_api.py
cp ../wearable_robot_mujoco/robot_control.py ../../Patient/$patient_name/robot_control_rpi5.py

CONTAINER_NAME=wearable_robot_mujoco
CONTOLLER_NODE=elbow_vel_cmd_node

echo "Starting $CONTOLLER_NODE in container $CONTAINER_NAME"
docker exec -it $CONTAINER_NAME bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install && source /wearable_ws/install/setup.sh && ros2 run wearable_robot_mujoco $CONTOLLER_NODE"

