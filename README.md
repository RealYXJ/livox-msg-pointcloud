# livox-msg-pointcloud
This is a ROS package (texted in ROS noetic) to convert Livox custom message to pcl pointcloud.

You can change the input topic name and output topic name in the launch file and then run 
```
roslaunch livox_msg_pointcloud livox_converter.launch
```

or just give them as parameters in the terminal
```
roslaunch livox_converter livox_converter.launch input_topic:=/custom/input_topic output_topic:=/custom/output_topic
```