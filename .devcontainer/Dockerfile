# Base image : ROS 2 Jazzy (Ubuntu 24.04)
FROM osrf/ros:jazzy-desktop

ARG USER_UID=1000
ARG USER_GID=1000
ARG USERNAME=niryo
ARG WORKSPACE_NAME=niryo_workshop

# Required Tools (Including Visualizing)
RUN apt-get update && apt-get install -y \
    nano \
    curl \
    wget \
    python3.12-venv \
    python3-pip \
    git \
    sudo \
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
    bash-completion \
    && rm -rf /var/lib/apt/lists/*

# Create non-root user with sudo privileges
# Handle existing group and user gracefully
RUN (groupadd --gid $USER_GID $USERNAME 2>/dev/null || true) \
    && (useradd --uid $USER_UID --gid $USER_GID -m $USERNAME 2>/dev/null || usermod -l $USERNAME -d /home/$USERNAME -m $(getent passwd $USER_UID | cut -d: -f1)) \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# X11 and GUI configuration
ENV DISPLAY=:0
ENV QT_X11_NO_MITSHM=1
ENV LIBGL_ALWAYS_INDIRECT=1

# Switch to non-root user
USER $USERNAME

# Set up workspace directory
WORKDIR /home/$USERNAME
RUN mkdir -p /home/$USERNAME/$WORKSPACE_NAME/src

# Clone repositories (as non-root user)
WORKDIR /home/$USERNAME/$WORKSPACE_NAME/src
RUN git clone https://github.com/NiryoRobotics/ned-ros2-driver.git && \
    git clone https://github.com/NiryoRobotics/roscon-2025.git

# Install ROS dependencies
WORKDIR /home/$USERNAME/$WORKSPACE_NAME
RUN sudo apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y --rosdistro=jazzy

# Set up Python virtual environment
RUN python3 -m venv /home/$USERNAME/$WORKSPACE_NAME/venv --system-site-packages && \
    /home/$USERNAME/$WORKSPACE_NAME/venv/bin/pip install --upgrade pip && \
    /home/$USERNAME/$WORKSPACE_NAME/venv/bin/pip install -r src/ned-ros2-driver/requirements.txt

# Configure shell environment
RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/$USERNAME/.bashrc && \
    echo "source /home/$USERNAME/$WORKSPACE_NAME/venv/bin/activate" >> /home/$USERNAME/.bashrc && \
    echo "source /home/$USERNAME/$WORKSPACE_NAME/install/setup.bash" >> /home/$USERNAME/.bashrc && \
    echo "export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST" >> /home/$USERNAME/.bashrc && \
    echo "cd /home/$USERNAME/$WORKSPACE_NAME" >> /home/$USERNAME/.bashrc

# Build ROS workspace
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && \
    colcon build --symlink-install"

# Expose port 9090 for web interfaces
EXPOSE 9090

CMD ["/bin/bash"]
