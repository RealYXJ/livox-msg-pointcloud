#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <livox_ros_driver/CustomMsg.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

ros::Publisher pub;

void livoxCallback(const livox_ros_driver::CustomMsg::ConstPtr& msg) {
    // Create a PCL PointCloud
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

    // Convert Livox data to PCL PointCloud
    for (const auto& point : msg->points) {
        pcl::PointXYZI pcl_point;
        pcl_point.x = point.x;
        pcl_point.y = point.y;
        pcl_point.z = point.z;
        pcl_point.intensity = point.reflectivity;
        cloud->points.push_back(pcl_point);
    }

    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = false;

    // Convert PCL PointCloud to ROS PointCloud2 message
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud, output);
    output.header.stamp = ros::Time::now();
    output.header.frame_id = cloud->header.frame_id;

    // Publish the PointCloud2 message
    pub.publish(output);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "livox_converter_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Get parameters
    std::string input_topic;
    std::string output_topic;
    private_nh.param<std::string>("input_topic", input_topic, "/livox/lidar");
    private_nh.param<std::string>("output_topic", output_topic, "/converted_pointcloud");

    pub = nh.advertise<sensor_msgs::PointCloud2>(output_topic, 10);
    ros::Subscriber sub = nh.subscribe(input_topic, 10, livoxCallback);

    ros::spin();

    return 0;
}
