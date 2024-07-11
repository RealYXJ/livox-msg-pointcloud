// utils.h
#pragma once

#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include "livox_ros_driver/CustomMsg.h"

typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;
// CustomMSG
    // Header header             # ROS standard message header
    // uint64 timebase           # The time of first point
    // uint32 point_num          # Total number of pointclouds
    // uint8  lidar_id           # Lidar device id number
    // uint8[3]  rsvd            # Reserved use
    // CustomPoint[] points      # Pointcloud data
// CustomPoint
    // uint32 offset_time      # offset time relative to the base time
    // float32 x               # X axis, unit:m
    // float32 y               # Y axis, unit:m
    // float32 z               # Z axis, unit:m
    // uint8 reflectivity      # reflectivity, 0~255
    // uint8 tag               # livox tag
    // uint8 line              # laser number in lidar
inline void livoxMsgToPCL(const livox_ros_driver::CustomMsgConstPtr& livox_msg_ptr,
                          pcl::PointCloud<pcl::PointXYZINormal>& pcl_in,
                          std::vector<livox_ros_driver::CustomMsgConstPtr>& livox_data, int TO_MERGE_CNT=1) {
    livox_data.push_back(livox_msg_ptr);
    if (livox_data.size() < TO_MERGE_CNT) return;
    
    for (size_t j = 0; j < livox_data.size(); j++) {
        auto& livox_msg = livox_data[j];
        auto time_end = livox_msg->points.back().offset_time;
        
        for (unsigned int i = 0; i < livox_msg->point_num; ++i) {
            pcl::PointXYZINormal pt;
            
            pt.x = livox_msg->points[i].x;
            pt.y = livox_msg->points[i].y;
            pt.z = livox_msg->points[i].z;
            float s = livox_msg->points[i].offset_time / (float)time_end;

            pt.intensity = livox_msg->points[i].line +livox_msg->points[i].reflectivity /10000.0 ; // The integer part is line number and the decimal part is timestamp
            pt.curvature = s*0.1;
            
            pcl_in.push_back(pt);
            // if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z) &&
            //     (pt.x != 0.0 || pt.y != 0.0 || pt.z != 0.0)) {
            //     pcl_ptr->push_back(pt);
            // } else {
            //     ROS_WARN("Invalid point detected in livoxMsgToPCL: (x=%.2f, y=%.2f, z=%.2f)", pt.x, pt.y, pt.z);
            // }
        }
    }
}

    // unsigned long timebase_ns = livox_data[0]->timebase;
    // ros::Time timestamp;
    // timestamp.fromNSec(timebase_ns);

    // sensor_msgs::PointCloud2 pcl_ros_msg;
    // pcl::toROSMsg(pcl_in, pcl_ros_msg);
    // pcl_ros_msg.header.stamp.fromNSec(timebase_ns);
    // pcl_ros_msg.header.frame_id = "/livox";
    // pub_pcl_out1.publish(pcl_ros_msg);
    // livox_data.clear();