#!/bin/sh

cd "$(dirname "$0")"

# copy xml from Index.dat to simulation folder
# TODO: use shell script instead of backend API
curl -X POST http://localhost:5000/api/xml/copy

#docker compose up -d --force-recreate

./stop.sh simulation

CONTAINER_NAME=wearable_robot_mujoco
SIMULATION_NODE=simulation_node

echo "Starting $SIMULATION_NODE in container $CONTAINER_NAME"
docker exec -it -d $CONTAINER_NAME bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install && source /wearable_ws/install/setup.sh && ros2 run wearable_robot_mujoco $SIMULATION_NODE"

# repeatly check if the simulation node is running
while true; do
  wmctrl -l | grep "MuJoCo : Elbow joint with single muscle" > /dev/null
  if [ $? -eq 0 ]; then
    echo "$SIMULATION_NODE is running"
    break
  else
    echo "Waiting for $SIMULATION_NODE to start..."
    sleep 0.01
  fi
done

DISPLAY_SIZE=$(xdpyinfo | grep 'dimensions:' | awk '{print $2}')
WIDTH=$(echo $DISPLAY_SIZE | cut -d'x' -f1)
HEIGHT=$(echo $DISPLAY_SIZE | cut -d'x' -f2)
WIDTH_TWO_THIRDS=$((WIDTH * 2 / 3))
WIDTH_ONE_THIRD=$((WIDTH / 3))
wmctrl -r "MuJoCo : Elbow joint with single muscle" -e "0,$WIDTH_TWO_THIRDS,0,$WIDTH_ONE_THIRD,$HEIGHT"
