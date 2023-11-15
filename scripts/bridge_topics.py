import argparse
import yaml
from docker_helper import RosDockerContainer


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('-t', '--topics-to-bridge', type=str, nargs='+')
    return parser


def get_ros1_topic_type_str(ros_bridge: RosDockerContainer, topic: str):
    command = f"rostopic info {topic}"
    res = ros_bridge.run(command, quiet=True)
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


def get_ros2_topic_type_str(ros_bridge: RosDockerContainer, topic: str):
    command = \
        f"source /opt/ros/foxy/setup.bash && " \
        f"ros2 topic info {topic}"
    res = ros_bridge.run(command, quiet=True)
    if res.returncode != 0:
        return None

    output = res.stdout
    line = next((line for line in output.split('\n') if line.startswith("Type: ")), None)
    if line is None:
        return None
    topic_type_str = line.rstrip()[len("Type: "):]
    return topic_type_str


def get_topic_type_str(ros_bridge: RosDockerContainer, topic: str):
    ros1_topic_type_str = get_ros1_topic_type_str(ros_bridge, topic)
    ros2_topic_type_str = get_ros2_topic_type_str(ros_bridge, topic)

    if ros1_topic_type_str is None and ros2_topic_type_str is None:
        raise RuntimeError(f"Cound not find topic {topic}.")
    if ros1_topic_type_str is not None and ros2_topic_type_str is not None and \
            ros1_topic_type_str != ros2_topic_type_str:
        raise RuntimeError(
            f"Topic {topic} has different types on two sides: "
            f"ROS1 - {ros1_topic_type_str}, "
            f"ROS2 - {ros2_topic_type_str}")

    return ros1_topic_type_str or ros2_topic_type_str


if __name__ == "__main__":
    parser = build_parser()
    args = parser.parse_args()

    topics_to_bridge = args.topics_to_bridge
    assert topics_to_bridge is not None

    ros_bridge = RosDockerContainer("ros_bridge:latest", "ros_bridge")
    ros_bridge.create_containter()

    topics_params = list()
    for topic in topics_to_bridge:
        if topic == "/tf_static":
            continue

        topic_type_str = get_topic_type_str(ros_bridge, topic)
        assert topic_type_str is not None

        queue_size = 1
        if topic == "/tf":
            queue_size = 100

        topic_param = dict()
        topic_param["topic"] = topic
        topic_param["type"] = topic_type_str
        topic_param["queue_size"] = queue_size
        topics_params.append(topic_param)

    if "/tf_static" in topics_to_bridge:
        ros_bridge.run_async(
            "cd ~/ros1_tf_static_bridge_ws && "
            "source install/setup.bash && "
            "ros2 run ros1_tf_static_bridge tf_static_bridge",
            session="tf_static_bridge")

    topics_params_str = yaml.dump(topics_params, default_flow_style=True).rstrip()
    ros_bridge.run(
        f"cd ~/ros1_bridge_ws && "
        f"source install/setup.bash && "
        f"rosparam set /topics '{topics_params_str}' && "
        f"ros2 run ros1_bridge parameter_bridge")

    if "/tf_static" in topics_to_bridge:
        ros_bridge.stop_session("tf_static_bridge")
