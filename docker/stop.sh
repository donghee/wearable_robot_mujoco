#!/bin/bash

if [ "$2" != "" ]; then
    echo "Usage: ./stop.sh" "[simulation|controller]"
    exit 1
fi

CONTAINER_NAME=wearable_robot_mujoco

if [ "$1" == "simulation" ]; then
  echo "Stopping simulation nodes..."
  docker exec -it -d $CONTAINER_NAME bash -c "ps aux | grep python3 | grep simulation_node | awk '{ print \$2 }' | xargs -n 1 kill -9"
fi

if [ "$1" == "controller" ]; then
  echo "Stopping controller nodes..."
  docker exec -it -d $CONTAINER_NAME bash -c "ps aux | grep python3 | grep -v simulation_node | awk '{ print \$2 }' | xargs -n 1 kill -9"
fi
