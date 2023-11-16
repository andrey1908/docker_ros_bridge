#!/bin/env python3

from docker_helper import RosDockerContainer

ros_bridge = RosDockerContainer("ros_bridge:latest", "ros_bridge")
ros_bridge.create_containter()
