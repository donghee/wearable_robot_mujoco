#!/bin/sh

pkill -f Wearable-Linux-Shipping
sleep 0.5

cd "$(dirname "$0")"

../../Wearable_Linux/Wearable.sh
