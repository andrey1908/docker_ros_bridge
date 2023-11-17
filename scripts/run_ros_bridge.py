import argparse
if __name__ == '__main__':
    from ros_bridge import RosBridgeContainer
else:
    from .ros_bridge import RosBridgeContainer


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('--topics', type=str, nargs='+')
    parser.add_argument('--types', type=str, nargs='+')

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

    ros_bridge.bridge_topics(args.topics, topic_types=args.types)
