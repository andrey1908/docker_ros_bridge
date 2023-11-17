FROM osrf/ros:noetic-desktop-full

ENV DEBIAN_FRONTEND noninteractive

ARG NUM_THREADS=1

# User setup
ARG USER=docker_ros_bridge
ARG UID=1000
ARG GID=1000
ARG PW=user

# Add user and his password
RUN useradd -m ${USER} --uid=${UID} && \
    echo "${USER}:${PW}" | chpasswd && \
    usermod -s /bin/bash ${USER} && \
    usermod -a -G sudo ${USER}

# Install dependencies
RUN apt-get update && apt-get install -y \
    clang \
    cmake \
    g++ \
    git \
    lsb-release \
    tmux \
    tar \
    vim \
    python3-pip && \
    rm -rf /var/lib/apt/lists/*

# Install ROS2 foxy
RUN apt-get update && apt-get install -y \
        software-properties-common \
        curl && \
    add-apt-repository universe && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt-get update && apt-get install -y \
        ros-foxy-desktop \
        ros-foxy-rmw-cyclonedds-cpp \
        python3-argcomplete \
        python3-colcon-common-extensions && \
    rm -rf /var/lib/apt/lists/*

# Make symlink from python3 to python
RUN PYTHON3_PATH=`which python3` && \
    ln -s $PYTHON3_PATH `dirname $PYTHON3_PATH`/python

RUN mkdir -p /home/${USER}/colcon_ws/src && \
    chown -R ${UID}:${GID} /home/${USER}

WORKDIR /home/${USER}
USER ${UID}:${GID}
