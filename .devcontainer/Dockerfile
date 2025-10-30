# Base image : ROS 2 Jazzy (Ubuntu 24.04)
FROM osrf/ros:jazzy-desktop

ARG WORKSPACE_NAME=niryo_workshop
ARG ROSCON_BRANCH=main

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

# X11 and GUI configuration
ENV DISPLAY=:0
ENV QT_X11_NO_MITSHM=1

# Set up workspace directory
WORKDIR /root
RUN mkdir -p /root/$WORKSPACE_NAME/src

# Clone repositories
WORKDIR /root/$WORKSPACE_NAME/src
RUN git clone https://github.com/NiryoRobotics/ned-ros2-driver.git && \
    git clone -b $ROSCON_BRANCH https://github.com/NiryoRobotics/roscon-2025.git

# Install ROS dependencies
WORKDIR /root/$WORKSPACE_NAME

# Set up Python virtual environment
RUN python3 -m venv /root/$WORKSPACE_NAME/venv --system-site-packages && \
    /root/$WORKSPACE_NAME/venv/bin/pip install --upgrade pip && \
    /root/$WORKSPACE_NAME/venv/bin/pip install -r src/ned-ros2-driver/requirements.txt

RUN apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y --rosdistro=jazzy

# Configure shell environment with network isolation
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo "source /root/$WORKSPACE_NAME/venv/bin/activate" >> /root/.bashrc && \
    echo "source /root/$WORKSPACE_NAME/install/setup.bash" >> /root/.bashrc && \
    echo "export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST" >> /root/.bashrc && \
    echo "cd /root/$WORKSPACE_NAME" >> /root/.bashrc

# Build ROS workspace
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && \
    colcon build --symlink-install"

# Expose port 9090 for web interfaces
EXPOSE 9090

CMD ["/bin/bash"]
