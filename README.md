**Installation:**

* Install dependencies:
```
# subprocess-tee
cd ~
git clone http://github.com/andrey1908/subprocess-tee
cd ~/subprocess-tee
pip install .

# docker_helper
cd ~
git clone http://github.com/andrey1908/docker_helper
cd ~/docker_helper
pip install .
```

* Clone repositories:
```
mkdir -p ~/ros_bridge
cd ~/ros_bridge
git clone http://github.com/andrey1908/docker_ros_bridge

mkdir -p ~/ros_bridge/ros_bridge_ws/src
cd ~/ros_bridge/ros_bridge_ws/src
git clone -b foxy https://github.com/ros2/ros1_bridge
git clone http://github.com/andrey1908/ros1_tf_static_bridge

mkdir -p ~/ros_bridge/ros1_msgs_ws/src
cd ~/ros_bridge/ros1_msgs_ws/src
# clone your custom messages and services for ROS1

mkdir -p ~/ros_bridge/ros2_msgs_ws/src
cd ~/ros_bridge/ros2_msgs_ws/src
# clone your custom messages and services for ROS2
```

* Build docker container:
```
cd ~/ros_bridge/docker_ros_bridge/docker
./build.sh
```

* Build ros_bridge from docker:
```
cd ~/ros_bridge/docker_ros_bridge/scripts
python3 run_ros_bridge.py --build
```

* Check that custom messages and services are built:
```
cd ~/ros_bridge/docker_ros_bridge/scripts
python3 run_ros_bridge.py --print-pairs
```
