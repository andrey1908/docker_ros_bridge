import argparse
try:
    from ros_bridge import RosBridgeContainer
except ImportError:
    from .ros_bridge import RosBridgeContainer


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('--topics', type=str, nargs='+', default=tuple())
    parser.add_argument('--topic-types', type=str, nargs='+')

    parser.add_argument('--services-2-to-1', type=str, nargs='+', default=tuple(), help="allows ROS2 to call ROS1 services")
    parser.add_argument('--services-2-to-1-types', type=str, nargs='+')

    parser.add_argument('--services-1-to-2', type=str, nargs='+', default=tuple(), help="allows ROS1 to call ROS2 services")
    parser.add_argument('--services-1-to-2-types', type=str, nargs='+')

    action = parser.add_mutually_exclusive_group()
    action.add_argument('--print-pairs', action='store_true')
    action.add_argument('--build', action='store_true')

    return parser


if __name__ == "__main__":
    parser = build_parser()
    args = parser.parse_args()

    ros_bridge = RosBridgeContainer()
    ros_bridge.create_containter()

    if args.build:
        ros_bridge.build_ros_bridge()
        exit(0)

    if args.print_pairs:
        ros_bridge.print_pairs()
        exit(0)

    if args.topics is None:
        exit(0)

    ros_bridge.run_bridge(
        topics=args.topics, topic_types=args.topic_types,
        services_2_to_1=args.services_2_to_1, services_2_to_1_types=args.services_2_to_1_types,
        services_1_to_2=args.services_1_to_2, services_1_to_2_types=args.services_1_to_2_types)
