import argparse
import os.path as osp
import yaml
from docker_helper import DockerMounts, RosDockerContainer
from typing import List


class RosBridgeMounts(DockerMounts):
    def __init__(self):
        super().__init__()
        self.add_folder(osp.join(osp.dirname(__file__), "../../ros_bridge_ws"), "/home/docker_ros_bridge/colcon_ws")
        self.add_folder(osp.join(osp.dirname(__file__), "../../ros1_msgs_ws"), "/home/docker_ros_bridge/ros1_msgs_ws")
        self.add_folder(osp.join(osp.dirname(__file__), "../../ros2_msgs_ws"), "/home/docker_ros_bridge/ros2_msgs_ws")


class RosBridgeContainer(RosDockerContainer):
    def __init__(self, image_name="ros_bridge:latest", container_name="ros_bridge"):
        super().__init__(image_name, container_name)

    def create_containter(self, mounts: RosBridgeMounts=None, net='host'):
        if mounts is None:
            mounts = RosBridgeMounts()
        if type(mounts) is not RosBridgeMounts:
            raise RuntimeError(f"Type of 'mounts' argument should be RosBridgeMounts, not {type(mounts)}")

        super().create_containter(mounts=mounts, net=net)

    def build_ros_bridge(self):
        self.run(
            "source /opt/ros/noetic/setup.bash && "
            "cd ~/ros1_msgs_ws && "
            "catkin_make")
        self.run(
            "source /opt/ros/foxy/setup.bash && "
            "cd ~/ros2_msgs_ws && "
            "colcon build")
        self.run(
            "source /opt/ros/noetic/setup.bash && "
            "source /opt/ros/foxy/setup.bash && "
            "source ~/ros1_msgs_ws/devel/setup.bash && "
            "source ~/ros2_msgs_ws/install/setup.bash && "
            "cd ~/colcon_ws && "
            "colcon build --cmake-force-configure")

    def bridge_topics(self, topics: List[str], topic_types: List[str]=None):
        if topic_types is None:
            topic_types = [""] * len(topics)
        assert len(topics) == len(topic_types)

        for i, (topic, topic_type) in enumerate(zip(topics, topic_types)):
            if topic == "/tf_static":
                continue
            if not topic_type:
                topic_type = self._get_topic_type_str(topic)
                topic_types[i] = topic_type

        if "/tf_static" in topics:
            self.run_async(
                "cd ~/colcon_ws && "
                "source install/setup.bash && "
                "ros2 run ros1_tf_static_bridge tf_static_bridge",
                session="tf_static_bridge")

        topics_param = self._get_topics_param(topics, topic_types)
        topics_param_str = yaml.dump(topics_param, default_flow_style=True).rstrip()
        self.run(
            f"source ~/ros1_msgs_ws/devel/setup.bash && "
            f"source ~/ros2_msgs_ws/install/setup.bash && "
            f"cd ~/colcon_ws && "
            f"source install/setup.bash && "
            f"rosparam set /topics '{topics_param_str}' && "
            f"ros2 run ros1_bridge parameter_bridge")

        if "/tf_static" in topics:
            self.stop_session("tf_static_bridge")

    def print_pairs(self):
        self.run(
            "source ~/ros1_msgs_ws/devel/setup.bash && "
            "source ~/ros2_msgs_ws/install/setup.bash && "
            "cd ~/colcon_ws && "
            "source install/setup.bash && "
            "ros2 run ros1_bridge dynamic_bridge --print-pairs")

    def _get_ros1_topic_type_str(self, topic: str):
        command = \
            f"source ~/ros1_msgs_ws/devel/setup.bash && " \
            f"rostopic info {topic}"
        res = self.run(command, quiet=True)
        if res.returncode != 0:
            return None

        output = res.stdout
        line = next((line for line in output.split('\n') if line.startswith("Type: ")), None)
        if line is None:
            return None
        topic_type_str = line.rstrip()[len("Type: "):]

        # convert ROS1 topic type name to ROS2
        topic_type_str = '/'.join(topic_type_str.split('/').insert('msg', 1))
        return topic_type_str

    def _get_ros2_topic_type_str(self, topic: str):
        command = \
            f"source ~/ros2_msgs_ws/install/setup.bash && " \
            f"ros2 topic info {topic}"
        res = self.run(command, quiet=True)
        if res.returncode != 0:
            return None

        output = res.stdout
        line = next((line for line in output.split('\n') if line.startswith("Type: ")), None)
        if line is None:
            return None
        topic_type_str = line.rstrip()[len("Type: "):]
        return topic_type_str

    def _get_topic_type_str(self, topic: str):
        ros1_topic_type_str = self._get_ros1_topic_type_str(topic)
        ros2_topic_type_str = self._get_ros2_topic_type_str(topic)

        if ros1_topic_type_str is None and ros2_topic_type_str is None:
            raise RuntimeError(f"Cound not find topic {topic}.")
        if ros1_topic_type_str is not None and ros2_topic_type_str is not None and \
                ros1_topic_type_str != ros2_topic_type_str:
            raise RuntimeError(
                f"Topic {topic} has different types on two sides: "
                f"ROS1 - {ros1_topic_type_str}, "
                f"ROS2 - {ros2_topic_type_str}")

        return ros1_topic_type_str or ros2_topic_type_str
    
    def _get_topics_param(self, topics, topic_types):
        topics_param = list()
        for topic, topic_type in zip(topics, topic_types):
            if topic == "/tf_static":
                continue

            queue_size = 1
            if topic == "/tf":
                queue_size = 100

            topic_param = dict()
            topic_param["topic"] = topic
            topic_param["type"] = topic_type
            topic_param["queue_size"] = queue_size
            topics_param.append(topic_param)
        return topics_param
