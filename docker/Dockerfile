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

COPY context/ /context
WORKDIR /context

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
        python3-argcomplete \
        python3-colcon-common-extensions && \
    rm -rf /var/lib/apt/lists/*

# Make symlink from python3 to python
RUN PYTHON3_PATH=`which python3` && \
    ln -s $PYTHON3_PATH `dirname $PYTHON3_PATH`/python

# Install ros1_bridge
ARG ROS1_BRIDGE_COMMIT=689a932499befbd1ec3cb273a1054430e55a43c3
RUN set -x && \
    tar -zxf ros1_bridge.tar.gz && \
    cd ros1_bridge && \
    git config --global --add safe.directory /context/ros1_bridge && \
    git checkout ${ROS1_BRIDGE_COMMIT} && \
    cd .. && \
    mkdir -p /home/${USER}/ros1_bridge_ws/src && \
    mv ros1_bridge /home/${USER}/ros1_bridge_ws/src && \
    cd /home/${USER}/ros1_bridge_ws && \
    /bin/bash -c " \
        source /opt/ros/noetic/setup.bash; \
        source /opt/ros/foxy/setup.bash; \
        colcon build --packages-select ros1_bridge --cmake-force-configure --cmake-args -DBUILD_TESTING=FALSE"

# Install ros1_tf_static_bridge
ARG ROS1_TF_STATIC_BRIDGE_COMMIT=e8e5cde6166c1b063f5595a1af777ac799b27b04
RUN set -x && \
    tar -zxf ros1_tf_static_bridge.tar.gz && \
    cd ros1_tf_static_bridge && \
    git config --global --add safe.directory /context/ros1_tf_static_bridge && \
    git checkout ${ROS1_TF_STATIC_BRIDGE_COMMIT} && \
    cd .. && \
    mkdir -p /home/${USER}/ros1_tf_static_bridge_ws/src && \
    mv ros1_tf_static_bridge /home/${USER}/ros1_tf_static_bridge_ws/src && \
    cd /home/${USER}/ros1_tf_static_bridge_ws && \
    /bin/bash -c " \
        source /opt/ros/noetic/setup.bash; \
        source /opt/ros/foxy/setup.bash; \
        colcon build"

RUN mkdir -p /home/${USER}/catkin_ws/src && \
    chown -R ${UID}:${GID} /home/${USER}

WORKDIR /home/${USER}
USER ${UID}:${GID}
