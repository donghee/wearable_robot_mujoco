#!/bin/bash
set -e

cd /app

if [ ! -d ".venv" ]; then
    echo "Creating virtual environment with Python 3.10..."
    uv venv --python 3.10 .venv
    
    echo "Installing Python packages..."
    source .venv/bin/activate
    uv add mujoco imageio numpy matplotlib cvxpy
else
    echo "Virtual environment already exists"
    source .venv/bin/activate
fi

echo "Starting application..."
exec "$@"
