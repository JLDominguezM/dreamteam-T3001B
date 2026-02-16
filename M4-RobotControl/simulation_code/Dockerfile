FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    python3-pip \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-xacro \
    libgl1-mesa-glx \
    libosmesa6-dev \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install mujoco matplotlib "numpy<2" dash pandas

WORKDIR /ros2_ws
RUN mkdir -p src

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc