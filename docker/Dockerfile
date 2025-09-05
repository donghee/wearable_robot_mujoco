FROM osrf/ros:humble-desktop

# Install uv
RUN apt-get update && apt-get install -y \
    curl python3-pip \
    && rm -rf /var/lib/apt/lists/*

RUN curl -LsSf https://astral.sh/uv/install.sh | sh
ENV PATH="/root/.local/bin:$PATH"

RUN pip install "numpy<2.0" mujoco imageio matplotlib cvxpy

# Create and set working directory
RUN mkdir -p /wearable_ws/src/sim_human
WORKDIR /wearable_ws/src/sim_human
# Copy sim_human files
COPY .. .

# Set Python version using uv
RUN uv python install 3.10

# Set environment variables for ROS
ENV ROS_DISTRO=humble
ENV ROS_PYTHON_VERSION=3

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

COPY ./docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
#CMD ["bash", "-c", "source /opt/ros/humble/setup.bash && python sim_human/simulation.py"]
