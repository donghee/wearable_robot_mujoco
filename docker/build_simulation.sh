#!/bin/sh

cd "$(dirname "$0")"

docker compose up -d --force-recreate

code ../wearable_robot_mujoco/task1.py
