FROM ubuntu:16.04

RUN \
  apt-get update && \
  apt-get -y install libgl1-mesa-glx libgl1-mesa-dri && \
  rm -rf /var/lib/apt/lists/*

# Setup environment
ENV DEBIAN_FRONTEND noninteractive
RUN apt-get update && apt-get install -y locales
RUN locale-gen en_US.UTF-8
ENV LANG en_US.UTF-8

# Install basic packages
RUN apt update && apt install -y \
    curl \
    sudo \
    tmux \
    openssh-server \
    software-properties-common \
    bash-completion \
    debian-keyring \
    debian-archive-keyring \
    tzdata \
    wget

# Update apt sources
RUN echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" \
    > /etc/apt/sources.list.d/gazebo-stable.list
RUN wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
RUN sh -c \
    'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# Install ros-kinetic and gazebo9
RUN apt update && \
    apt install -y \
    ros-kinetic-desktop \
    gazebo9 \
    libgazebo9-dev \
    ros-kinetic-gazebo9-ros-pkgs \
    ros-kinetic-gazebo9-ros-control

# Install other ROS packages
RUN apt update && \
    apt install -y \
    ros-kinetic-xacro \
    ros-kinetic-amcl \
    ros-kinetic-camera-calibration \
    ros-kinetic-controller-manager \
    ros-kinetic-ecl-threads \
    ros-kinetic-hardware-interface \
    ros-kinetic-joint-limits-interface \
    ros-kinetic-move-base \
    ros-kinetic-pcl-ros \
    ros-kinetic-robot-localization \
    ros-kinetic-transmission-interface \
    ros-kinetic-yocs-controllers

# Create a user with passwordless sudo
RUN adduser -uid 1001 --gecos "Development User" --disabled-password ivanpauno
RUN adduser ivanpauno sudo
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
RUN echo "export QT_X11_NO_MITSHM=1" >> /home/ivanpauno/.bashrc

RUN usermod -a -G video ivanpauno
RUN usermod -a -G dialout ivanpauno
USER ivanpauno

WORKDIR /home/ivanpauno

