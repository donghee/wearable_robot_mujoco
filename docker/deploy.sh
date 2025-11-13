#!/bin/sh

cd "$(dirname "$0")"

patient_name=$(cat ../../Index.dat| head -n 1)
cp ../wearable_robot_mujoco/robot_control.py ../../Patient/$patient_name/robot_control_rpi5.py
