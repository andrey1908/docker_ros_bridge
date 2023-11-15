#!/bin/bash

set -e

docker_dir=$(dirname $0)

to_archive () {
    tar --remove-files -czf $1.tar.gz $1 
}

if [[ -d "$docker_dir/context" ]]
then
    echo "$(realpath ${docker_dir}/context) already exists. Delete it if you want to update docker context."
else
    mkdir $docker_dir/context
    cd $docker_dir/context
    echo "Download docker context to ${PWD}"

    echo "Cloning"
    git clone -b foxy https://github.com/ros2/ros1_bridge.git
    to_archive ros1_bridge
    git clone https://github.com/andrey1908/ros1_tf_static_bridge.git
    to_archive ros1_tf_static_bridge
fi

