#include <ros/ros.h>
#include "utils.h"

ros::Publisher pub;

std::vector<livox_ros_driver::CustomMsgConstPtr> livox_data;

int TO_MERGE_CNT = 50;

void livoxCallback(const livox_ros_driver::CustomMsg::ConstPtr& msg) 
{   
    pcl::PointCloud<pcl::PointXYZINormal> pcl_in;

    livoxMsgToPCL(msg, pcl_in, livox_data, TO_MERGE_CNT);

    if (livox_data.size() < TO_MERGE_CNT) return;
    unsigned long timebase_ns = livox_data[0]->timebase;
    ros::Time timestamp;
    timestamp.fromNSec(timebase_ns);

    sensor_msgs::PointCloud2 pcl_ros_msg;
    pcl::toROSMsg(pcl_in, pcl_ros_msg);
    pcl_ros_msg.header.stamp.fromNSec(timebase_ns);
    pcl_ros_msg.header.frame_id = msg->header.frame_id;
    pub.publish(pcl_ros_msg);
    livox_data.clear();
    pcl_in.clear();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "livox_converter_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Get parameters
    std::string input_topic;
    std::string output_topic;
	int scan_merge_count;
    private_nh.param<std::string>("input_topic", input_topic, "/livox/lidar");
    private_nh.param<std::string>("output_topic", output_topic, "/converted_pointcloud");
    private_nh.param("scan_merge_count", scan_merge_count, 10); 

	TO_MERGE_CNT = scan_merge_count;
    pub = nh.advertise<sensor_msgs::PointCloud2>(output_topic, 10);
    ros::Subscriber sub = nh.subscribe(input_topic, 10, livoxCallback);
	ROS_INFO(">>>>>> Params Set: ");
    ROS_INFO("                  input_topic: %s", input_topic.c_str());
    ROS_INFO("                 output_topic: %s", output_topic.c_str());
    ROS_INFO("             scan_merge_count: %d", scan_merge_count);
    ros::spin();

    return 0;
}
