#!/bin/sh

pkill -f Wearable-Linux-Shipping
sleep 0.5

cd "$(dirname "$0")"

patient_name=$(cat ../../Index.dat| head -n 1)
echo "Patient name: $patient_name"
cp ../../Patient/$patient_name/result.csv ../../Wearable_Linux/Wearable/CSV/elbow_joint_quaternion_extended_20260114_171949.csv

../../Wearable_Linux/Wearable.sh
