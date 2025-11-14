#!/bin/bash

cd "$(dirname "$0")"

# Create Index.dat to prepare frontend 
touch ../../Index.dat
mkdir -p ../../Patient

docker compose up -d --build
