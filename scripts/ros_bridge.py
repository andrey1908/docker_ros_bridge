import os
import os.path as osp
import yaml
from docker_helper import DockerMounts, RosDockerContainer


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
        os.makedirs(osp.join(osp.dirname(__file__), "../../ros1_msgs_ws/src"), exist_ok=True)
        os.makedirs(osp.join(osp.dirname(__file__), "../../ros2_msgs_ws/src"), exist_ok=True)

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

    def run_bridge(self,
            topics=tuple(), topic_types=None,
            services_2_to_1=tuple(), services_2_to_1_types=None,
            services_1_to_2=tuple(), services_1_to_2_types=None):
        topics_param_str = self._prepare_topics(topics, topic_types)
        services_2_to_1_param_str = self._prepare_services_2_to_1(services_2_to_1, services_2_to_1_types)
        services_1_to_2_param_str = self._prepare_services_1_to_2(services_1_to_2, services_1_to_2_types)

        if "/tf_static" in topics:
            self.run_async(
                "cd ~/colcon_ws && "
                "source install/setup.bash && "
                "ros2 run ros1_tf_static_bridge tf_static_bridge",
                session="tf_static_bridge")

        self.run(
            f"source ~/ros1_msgs_ws/devel/setup.bash && "
            f"source ~/ros2_msgs_ws/install/setup.bash && "
            f"cd ~/colcon_ws && "
            f"source install/setup.bash && "
            f"rosparam set /topics '{topics_param_str}' && "
            f"rosparam set /services_2_to_1 '{services_2_to_1_param_str}' && "
            f"rosparam set /services_1_to_2 '{services_1_to_2_param_str}' && "
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
        
    def _prepare_topics(self, topics, topic_types):
        if topic_types is None:
            topic_types = [""] * len(topics)
        assert len(topics) == len(topic_types)

        for i, (topic, topic_type) in enumerate(zip(topics, topic_types)):
            if topic == "/tf_static":
                continue
            if not topic_type:
                topic_type = self._get_topic_type_str(topic)
                if topic_type is None:
                    raise RuntimeError(f"Could not find topic {topic}.")
                topic_types[i] = topic_type

        topics_param = self._get_topics_param(topics, topic_types)
        topics_param_str = yaml.dump(topics_param, default_flow_style=True).rstrip()

        return topics_param_str
    
    def _prepare_services_2_to_1(self, services_2_to_1, services_2_to_1_types):
        if services_2_to_1_types is None:
            services_2_to_1_types = [""] * len(services_2_to_1)
        assert len(services_2_to_1) == len(services_2_to_1_types)

        for i, (service, service_type) in enumerate(zip(services_2_to_1, services_2_to_1_types)):
            if not service_type:
                service_type = self._get_ros1_service_type_str(service)
                if service_type is None:
                    raise RuntimeError(f"Could not find ROS1 service {service}.")
                services_2_to_1_types[i] = service_type

        services_2_to_1_param = self._get_services_2_to_1_param(services_2_to_1, services_2_to_1_types)
        services_2_to_1_param_str = yaml.dump(services_2_to_1_param, default_flow_style=True).rstrip()

        return services_2_to_1_param_str
    
    def _prepare_services_1_to_2(self, services_1_to_2, services_1_to_2_types):
        if services_1_to_2_types is None:
            services_1_to_2_types = [""] * len(services_1_to_2)
        assert len(services_1_to_2) == len(services_1_to_2_types)

        for i, (service, service_type) in enumerate(zip(services_1_to_2, services_1_to_2_types)):
            if not service_type:
                service_type = self._get_ros2_service_type_str(service)
                if service_type is None:
                    raise RuntimeError(f"Could not find ROS2 service {service}.")
                services_1_to_2_types[i] = service_type

        services_1_to_2_param = self._get_services_1_to_2_param(services_1_to_2, services_1_to_2_types)
        services_1_to_2_param_str = yaml.dump(services_1_to_2_param, default_flow_style=True).rstrip()

        return services_1_to_2_param_str

    def _get_ros1_topic_type_str(self, topic: str):
        command = \
            f"source ~/ros1_msgs_ws/devel/setup.bash && " \
            f"rostopic info {topic}"
        res = self.run(command, quiet=True)
        if res.returncode != 0:
            return None

        output = res.stdout
        line = next((line for line in output.split('\n') if line.startswith("Type: ")), None)
        assert line is not None, "This should not happen"
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
        assert line is not None, "This should not happen"
        topic_type_str = line.rstrip()[len("Type: "):]
        return topic_type_str

    def _get_topic_type_str(self, topic: str):
        ros1_topic_type_str = self._get_ros1_topic_type_str(topic)
        ros2_topic_type_str = self._get_ros2_topic_type_str(topic)

        if ros1_topic_type_str is not None and ros2_topic_type_str is not None and \
                ros1_topic_type_str != ros2_topic_type_str:
            raise RuntimeError(
                f"Topic {topic} has different types on two sides: "
                f"ROS1 - {ros1_topic_type_str}, "
                f"ROS2 - {ros2_topic_type_str}")

        return ros1_topic_type_str or ros2_topic_type_str

    def _get_ros1_service_type_str(self, service: str):
        command = \
            f"source ~/ros1_msgs_ws/devel/setup.bash && " \
            f"rosservice info {service}"
        res = self.run(command, quiet=True)
        if res.returncode != 0:
            return None

        output = res.stdout
        line = next((line for line in output.split('\n') if line.startswith("Type: ")), None)
        assert line is not None, "This should not happen"
        service_type_str = line.rstrip()[len("Type: "):]
        return service_type_str
    
    def _get_ros2_service_type_str(self, service: str):
        command = \
            f"source ~/ros2_msgs_ws/install/setup.bash && " \
            f"ros2 service type {service}"
        res = self.run(command, quiet=True)
        if res.returncode != 0:
            return None

        output = res.stdout
        lines = [line for line in output.split('\n') if len(line.rstrip()) != 0]
        assert len(lines) > 0, "This should not happen"
        service_type_str = lines[-1].rstrip()
        return service_type_str

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

    def _get_services_2_to_1_param(self, services_2_to_1, services_2_to_1_types):
        services_2_to_1_param = list()
        for service, service_type in zip(services_2_to_1, services_2_to_1_types):
            service_param = dict()
            service_param["service"] = service
            service_param["type"] = service_type
            services_2_to_1_param.append(service_param)
        return services_2_to_1_param

    def _get_services_1_to_2_param(self, services_1_to_2, services_1_to_2_types):
        services_1_to_2_param = list()
        for service, service_type in zip(services_1_to_2, services_1_to_2_types):
            service_param = dict()
            service_param["service"] = service
            service_param["type"] = service_type
            services_1_to_2_param.append(service_param)
        return services_1_to_2_param
