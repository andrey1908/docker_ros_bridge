#!/bin/bash

cd $(dirname $0)

if ! [[ -d "context" ]]
then
    bash context.sh
    if [ $? -ne 0 ]; then
        rm -rf context
        exit -1
    fi
fi

NUM_THREADS=${1:-$(nproc)}

docker build . \
    --build-arg UID=$(id -g) \
    --build-arg GID=$(id -g) \
    --build-arg NUM_THREADS=${NUM_THREADS} \
    --build-arg RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION} \
    --build-arg ROS_DOMAIN_ID=${ROS_DOMAIN_ID} \
    --build-arg ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY} \
    -f Dockerfile \
    -t ros_bridge:latest
