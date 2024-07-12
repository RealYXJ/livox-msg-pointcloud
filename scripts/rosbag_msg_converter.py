import rosbag
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from livox_ros_driver.msg import CustomMsg
import numpy as np
import argparse
import signal
import sys
import std_msgs.msg
import logging
import time
import tracemalloc
from tqdm import tqdm

def setup_logging():
    logging.basicConfig(level=logging.INFO,
                        format='%(asctime)s [%(levelname)s] %(message)s',
                        handlers=[
                            logging.StreamHandler(sys.stdout)
                        ])

def livoxMsgsToPointCloud2(livox_msgs, point_cloud2_msg):
    frame_id = livox_msgs[0].header.frame_id
    total_points = sum(msg.point_num for msg in livox_msgs)
    all_points = np.zeros((total_points, 5), dtype=np.float32)
    idx = 0
    
    for livox_msg in livox_msgs:
        time_end = livox_msg.points[-1].offset_time
        num_points = livox_msg.point_num
        
        points = np.zeros((num_points, 5), dtype=np.float32)
        for i, point in enumerate(livox_msg.points):
            points[i, 0] = point.x
            points[i, 1] = point.y
            points[i, 2] = point.z
            points[i, 3] = point.line + point.reflectivity / 10000.0
            points[i, 4] = point.offset_time / float(time_end) * 0.1
        
        all_points[idx:idx+num_points] = points
        idx += num_points
    
    header = std_msgs.msg.Header()
    header.stamp = livox_msgs[0].header.stamp
    header.frame_id = frame_id
    point_cloud2_msg.header = header
    point_cloud2_msg.height = 1
    point_cloud2_msg.width = total_points
    point_cloud2_msg.fields = [
        point_cloud2.PointField('x', 0, point_cloud2.PointField.FLOAT32, 1),
        point_cloud2.PointField('y', 4, point_cloud2.PointField.FLOAT32, 1),
        point_cloud2.PointField('z', 8, point_cloud2.PointField.FLOAT32, 1),
        point_cloud2.PointField('intensity', 12, point_cloud2.PointField.FLOAT32, 1),
        point_cloud2.PointField('curvature', 16, point_cloud2.PointField.FLOAT32, 1),
    ]
    point_cloud2_msg.is_bigendian = False
    point_cloud2_msg.point_step = 20
    point_cloud2_msg.row_step = point_cloud2_msg.point_step * total_points
    point_cloud2_msg.is_dense = True
    point_cloud2_msg.data = all_points.tobytes()

def signal_handler(sig, frame):
    logging.info("KeyboardInterrupt caught. Exiting gracefully.")
    sys.exit(0)

def log_memory_usage():
    current, peak = tracemalloc.get_traced_memory()
    logging.info(f"Current memory usage: {current / 10**6:.2f} MB; Peak: {peak / 10**6:.2f} MB")

if __name__ == "__main__":
    setup_logging()
    
    parser = argparse.ArgumentParser(description="Convert Livox CustomMsg to PointCloud2 in a ROS bag.")
    parser.add_argument('input_bag', help='Input ROS bag file')
    parser.add_argument('output_bag', help='Output ROS bag file')
    parser.add_argument('scan_merge_count', type=int, help='Number of messages to merge')
    parser.add_argument('save_interval', type=int, help='Number of processed messages to save periodically')
    parser.add_argument('topics', nargs='*', help='Specific topics to process. If none, process all topics of the specified type.')
    args = parser.parse_args()

    signal.signal(signal.SIGINT, signal_handler)

    tracemalloc.start()
    
    input_bag = args.input_bag
    output_bag = args.output_bag
    scan_merge_count = args.scan_merge_count
    save_interval = args.save_interval
    topics_to_process = args.topics if args.topics else None
    target_topic_type = 'livox_ros_driver/CustomMsg'
    livox_data = []
    processed_messages = []

    start_time = time.time()
    
    try:
        logging.info(f"Reading from {input_bag}")

        with rosbag.Bag(input_bag, 'r') as inbag, rosbag.Bag(output_bag, 'w') as outbag:
            if not topics_to_process:
                topics_to_process = [
                    topic for topic, info in inbag.get_type_and_topic_info()[1].items() if info.msg_type == target_topic_type
                ]

            total_messages = inbag.get_message_count(topic_filters=topics_to_process)
            pbar = tqdm(total=total_messages, desc='Processing messages')

            for topic, msg, t in inbag.read_messages():
                pbar.update(1)
                if topic in topics_to_process and inbag.get_type_and_topic_info()[1][topic].msg_type == target_topic_type:
                    livox_data.append(msg)
                    if len(livox_data) >= scan_merge_count:
                        point_cloud2_msg = PointCloud2()
                        livoxMsgsToPointCloud2(livox_data, point_cloud2_msg)
                        livox_data = []
                        processed_messages.append((topic, point_cloud2_msg, t))
                        # logging.info(f"Processed and prepared merged point cloud for topic {topic} at time {t}")
                        log_memory_usage()

                        # Save periodically
                        if len(processed_messages) >= save_interval:
                            logging.info(f"Saving {len(processed_messages)} processed messages to {output_bag}")
                            for p_topic, p_msg, p_t in processed_messages:
                                outbag.write(p_topic, p_msg, p_t)
                            processed_messages = []
                else:
                    processed_messages.append((topic, msg, t))

                    # Save periodically
                    if len(processed_messages) >= save_interval:
                        logging.info(f"Saving {len(processed_messages)} processed messages to {output_bag}")
                        for p_topic, p_msg, p_t in processed_messages:
                            outbag.write(p_topic, p_msg, p_t)
                        processed_messages = []

            # Final save for remaining messages
            if processed_messages:
                logging.info(f"Final save of {len(processed_messages)} remaining processed messages to {output_bag}")
                for p_topic, p_msg, p_t in processed_messages:
                    outbag.write(p_topic, p_msg, p_t)

            pbar.close()

        logging.info("Finished processing the rosbag.")
    except KeyboardInterrupt:
        signal_handler(None, None)
    
    end_time = time.time()
    elapsed_time = end_time - start_time
    logging.info(f"Total processing time: {elapsed_time:.2f} seconds")
    tracemalloc.stop()
