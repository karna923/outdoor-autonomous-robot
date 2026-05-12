FROM ros:jazzy-ros-base

RUN apt-get update && apt-get install -y \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox \
  ros-jazzy-robot-localization \
  ros-jazzy-rplidar-ros \
  python3-colcon-common-extensions \
  && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws
COPY ../ros2_ws/src ./src
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build"
