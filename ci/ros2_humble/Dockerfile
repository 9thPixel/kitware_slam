FROM osrf/ros:humble-desktop-full
LABEL maintainer="Arthur Bourbousson arthur.bourbousson@kitware.com"

WORKDIR /tmp

RUN apt update

# Install Slam dependencies
COPY install_deps.sh install_deps.sh
RUN sh install_deps.sh

COPY install_ceres.sh install_ceres.sh
RUN sh install_ceres.sh

COPY install_nanoflann.sh install_nanoflann.sh
RUN sh install_nanoflann.sh

COPY install_g2o.sh install_g2o.sh
RUN sh install_g2o.sh

# Install ROS dependencies
RUN apt install -y ros-$ROS_DISTRO-velodyne

# Graphic display on host
RUN apt -y install libgl1-mesa-glx libgl1-mesa-dri

# Erase apt cache
RUN rm -rf /var/lib/apt/lists/*

WORKDIR /root