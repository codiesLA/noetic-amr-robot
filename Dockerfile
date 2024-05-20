FROM osrf/ros:noetic-desktop-full

ARG DEBIAN_FRONTEND=noninteractive

# Install apt packages
RUN apt-get update \
    && apt-get install -y \
    # && ros-${ROS_DISTRO}-ros-tutorials \
    git \
    nano \
    vim \
    wget \
    tmux \
    terminator \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/* 

# Install ROS packages
RUN apt-get update && apt-get install -y \
    ros-noetic-joy ros-noetic-teleop-twist-joy \
    ros-noetic-teleop-twist-keyboard\
    ros-noetic-amcl ros-noetic-map-server \
    ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro \
    ros-noetic-compressed-image-transport ros-noetic-rqt* ros-noetic-rviz \
    ros-noetic-gmapping ros-noetic-navigation\
    apt-get clean && rm -rf /var/lib/apt/lists/*
    

COPY src/ /dev_machine-docker/src

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create a non-root user
RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  && mkdir /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config

# Set up sudo
RUN apt-get update \
  && apt-get install -y sudo \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
  && chmod 0440 /etc/sudoers.d/$USERNAME \ 
  && rm -rf /var/lib/apt/lists/*

COPY entrypoint.sh /entrypoint.sh
COPY bashrc /home/${USERNAME}/.bashrc

# RUN -chmod x+ run_docker.bash

ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]

CMD ["bash"]


