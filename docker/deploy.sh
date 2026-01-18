#!/bin/sh

cd "$(dirname "$0")"

# Copy task module to patient
patient_name=$(cat ../../Index.dat| head -n 1)
cp ../wearable_robot_mujoco/task.py ../../Patient/$patient_name/task.py
cp ../wearable_robot_mujoco/wearable_robot_api.py ../../Patient/$patient_name/wearable_robot_api.py
cp ../wearable_robot_mujoco/robot_control.py ../../Patient/$patient_name/robot_control_rpi5.py

# Deploy task module to Raspberry Pi 5
#scp ../../Patient/$patient_name/task.py raspberrypi.local:dynamixel_ws/src/wearable_robot_upper_limb/wearable_robot_upper_limb/
#scp ../../Patient/$patient_name/robot_control_rpi5.py raspberrypi.local:dynamixel_ws/src/wearable_robot_upper_limb/wearable_robot_upper_limb/
#scp ../../Patient/$patient_name/wearable_robot_api.py raspberrypi.local:dynamixel_ws/src/wearable_robot_upper_limb/wearable_robot_upper_limb/
