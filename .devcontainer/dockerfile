# Base image : ROS 2 Jazzy (Ubuntu 24.04)
FROM osrf/ros:jazzy-desktop

# Required Tools (Including Visualizing)
RUN apt-get update && apt-get install -y \
    nano \
    python3.12-venv \
    python3-pip \
    git \
    ros-dev-tools \
    avahi-daemon \
    libnss-mdns \
    libogre-1.12-dev \
    libgles2-mesa-dev \
    libgl1 \
    libxkbcommon-x11-0 \
    mesa-utils \
    libgl1-mesa-dri \
    openssh-client \
    && rm -rf /var/lib/apt/lists/*

# X11
ENV DISPLAY=:0
ENV QT_X11_NO_MITSHM=1

# Clone thomas' ros2driver
WORKDIR /root
RUN mkdir -p /root/ros2_drivers_ws/src && \
    cd /root/ros2_drivers_ws/src && \
    git clone https://github.com/NiryoRobotics/ned-ros2-driver.git

# Rosdeping
WORKDIR /root/ros2_drivers_ws
RUN apt-get update && rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y


# Piping
RUN python3 -m venv /root/ros2_drivers_ws/venv --system-site-packages && \
    /root/ros2_drivers_ws/venv/bin/pip install --upgrade pip && \
    /root/ros2_drivers_ws/venv/bin/pip install -r src/ned-ros2-driver/requirements.txt

# Venving
RUN echo "source /root/ros2_drivers_ws/venv/bin/activate" >> /root/.bashrc

# Expose port 9090
EXPOSE 9090

# Idle
CMD ["bash"]
