import rosbag
import argparse

def get_bag_topic_types(input_bag):
    with rosbag.Bag(input_bag, 'r') as bag:
        topic_types = {}
        for topic, msg, t in bag.read_messages():
            if topic not in topic_types:
                topic_types[topic] = bag.get_type_and_topic_info()[1][topic].msg_type
        return topic_types

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Get all topic types from a ROS bag.")
    parser.add_argument('input_bag', help='Input ROS bag file')
    args = parser.parse_args()

    topic_types = get_bag_topic_types(args.input_bag)

    for topic, msg_type in topic_types.items():
        print(f"Topic: {topic}, Type: {msg_type}")
