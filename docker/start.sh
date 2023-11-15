#!/bin/bash

docker_dir=$(dirname $0)

docker run -it -d --rm \
    --ipc host \
    --gpus all \
    --env="NVIDIA_DRIVER_CAPABILITIES=all" \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION" \
    --env="ROS_DOMAIN_ID=$ROS_DOMAIN_ID" \
    --env="ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY" \
    --privileged \
    --net "host" \
    --name ros_bridge \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v /etc/timezone:/etc/timezone:ro \
    -v /etc/localtime:/etc/localtime:ro \
    ros_bridge:latest
