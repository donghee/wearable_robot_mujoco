FROM osrf/ros:humble-desktop

# Install uv
RUN apt-get update && apt-get install -y \
    curl \
    && rm -rf /var/lib/apt/lists/*

RUN curl -LsSf https://astral.sh/uv/install.sh | sh
ENV PATH="/root/.local/bin:$PATH"

# Set working directory
WORKDIR /app

# Copy app files
COPY .. .

# Set Python version using uv
RUN uv python install 3.10

# Set environment variables for ROS
ENV ROS_DISTRO=humble
ENV ROS_PYTHON_VERSION=3

COPY ./docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash", "-c", "source /opt/ros/humble/setup.bash && python simulation.py"]
