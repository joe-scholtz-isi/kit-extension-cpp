# OmniGraph Extension [omni.isaac.ros2_cpp_custom_og_node]

## Install

Tested on Debian-based Linux.

- Git clone the [kit-extension-template-cpp](kit-extension-template-cpp) repository, copy this directory to `source/extensions`
- Run `kit-extension-template-cpp/build.sh`
- Add extention build path (`kit-extension-template-cpp/_build/{plataform}/release/exts`) to Isaac Sim Extension Search Paths in Window->Extensions->"Three bar icon ☰"->Settings. And enable the omni.isaac.ros2_cpp_custom_og_node extension inside Isaac Sim going to Window->Extensions->THIRD PARTY. NB.: for me `{plataform}` is `linux-x86_64`.

## Nodes

- omni.isaac.ros2_cpp_custom_og_node.ROS2RTXLiDARPublishPointCloudNode: Takes RTX LiDAR x,y,z,intensity arrays as input and publish a PointCloud2ROS2 message.
- omni.isaac.ros2_cpp_custom_og_node.ROS2ExampleNode: This node publishes a given string as a ROS2 message.

## Resources

Here are the resources base on which the nodes were created:

- [Kit C++ Extension Template](https://docs.omniverse.nvidia.com/kit/docs/kit-extension-template-cpp/latest/index.html);
- [ROS 2 Custom C++ OmniGraph Node](https://docs.omniverse.nvidia.com/isaacsim/latest/ros2_tutorials/tutorial_ros2_omnigraph_cpp_node.html);
- ROS 2 [Client libraries](https://docs.ros.org/en/rolling/Concepts/Basic/About-Client-Libraries.html);
- ROS 2 [Internal ROS 2 interfaces](https://docs.ros.org/en/rolling/Concepts/Advanced/About-Internal-Interfaces.html);
- [ROS 2 PointCloud2 Raw Message Definition](https://docs.ros2.org/latest/api/sensor_msgs/msg/PointCloud2.html)
- `{ROS 2 path}/include/sensor_msgs/sensor_msgs/point_cloud_conversion.hpp`. Where `{ROS 2 path}` is usually `/opt/ros/$ROS_DISTRO`;
- `{ROS 2 path}/include/sensor_msgs/sensor_msgs/msg/detail/point_cloud2__functions.c`;
- `{ROS 2 path}/include/sensor_msgs/sensor_msgs/msg/detail/point_field__functions.c`;
- `{ROS 2 path}/include/std_msgs/std_msgs/msg/detail/string__functions.c`;
- `{ROS 2 path}/include/rosidl_runtime_c/rosidl_runtime_c/primitives_sequence_functions.h`;
- [Intensity data PointCloud2](https://forums.developer.nvidia.com/t/intensity-data-pointcloud2/263133)

## TODO

- Maybe implement another version of ROS2RTXLiDARPublishPointCloudNode that takes the render product path direclty as an input and computes x,y,z,intensity from it?
