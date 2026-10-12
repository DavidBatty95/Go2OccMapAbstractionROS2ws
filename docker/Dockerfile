FROM osrf/ros:humble-desktop-full@sha256:1db1e4e941d4f77fab55bcd479158a75273329335e86ec468bff280859e8178c
ENV DEBIAN_FRONTEND=noninteractive ROS_DISTRO=humble PYTHONDONTWRITEBYTECODE=1
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip python3-colcon-common-extensions libglfw3 libgl1-mesa-dri libegl1 \
    ros-humble-rmw-cyclonedds-cpp && rm -rf /var/lib/apt/lists/*
COPY docker/requirements.txt /tmp/requirements.txt
RUN python3 -m pip install --no-cache-dir -r /tmp/requirements.txt && \
    python3 -m pip install --no-cache-dir torch==2.6.0 --index-url https://download.pytorch.org/whl/cpu
WORKDIR /opt/go2_ws
COPY src/ src/
RUN . /opt/ros/humble/setup.sh && colcon build --merge-install
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ARG SOURCE_SHA256=unknown
LABEL org.opencontainers.image.source="https://github.com/DavidBatty95/Go2OccMapAbstractionROS2ws" \
      go2.runtime_source_sha256="${SOURCE_SHA256}"
WORKDIR /workspace
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "bringup_go2", "simulation.launch.py"]
