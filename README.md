# livox-msg-pointcloud
This is a ROS package (tested in ROS noetic) to convert Livox custom message to pcl pointcloud.

```
livox_msg_pointcloud/
├── CMakeLists.txt
├── README.md
├── include
│   └── utils.h
├── launch
│   └── livox_converter.launch
├── package.xml
└── src
    └── livox_msg_pointcloud_node.cpp
```

The main conversion code is inside `utils.h` file and  `ivox_msg_pointcloud_node.cpp` is a ROS node wrapper. You can also directly use the header file in our coude if you don't use ROS.

You can change the input topic name and output topic name in the launch file and then run 
```
roslaunch livox_msg_pointcloud livox_converter.launch
```

or just give them as parameters in the terminal
```
roslaunch livox_msg_pointcloud livox_converter.launch input_topic:=/custom/input_topic  output_topic:=/custom/output_topic  scan_merge_count:=1
```
Parameters explained:
```
input_topic # the original Livox LiDAR point cloud in custom msg
output_topoic # the converted LiDAR point cloud in PCL pointcloud format
scan_merge_count #the original point cloud is in high frequency but very sparse. We can choose to low the frequency to get dense point cloud.
```