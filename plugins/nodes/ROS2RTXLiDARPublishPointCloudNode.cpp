// Copyright (c) 2022, NVIDIA CORPORATION. All rights reserved.
//
// NVIDIA CORPORATION and its licensors retain all intellectual property
// and proprietary rights in and to this software, related documentation
// and any modifications thereto.  Any use, reproduction, disclosure or
// distribution of this software and related documentation without an express
// license agreement from NVIDIA CORPORATION is strictly prohibited.
//
#include <ROS2RTXLiDARPublishPointCloudNodeDatabase.h>
#include <string>

// In this example, we will publish a string message with an OmniGraph Node
#include "rosidl_runtime_c/string_functions.h"
#include "sensor_msgs/msg/point_cloud2.h"
#include "std_msgs/msg/header.h"
#include "std_msgs/msg/string.h"

// ROS includes for creating nodes, publishers etc.
#include "rcl/rcl.h"

// Helpers to explicit shorten names you know you will use
using omni::graph::core::BaseDataType;
using omni::graph::core::Type;

namespace custom {
namespace ros2_cpp {
namespace omnigraph_node {

class ROS2RTXLiDARPublishPointCloudNode {
public:
  static bool compute(ROS2RTXLiDARPublishPointCloudNodeDatabase &db) {
    auto &state = db.internalState<ROS2RTXLiDARPublishPointCloudNode>();

    if (!state.pub_created) {
      rcl_context_t context = rcl_get_zero_initialized_context();
      rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
      rcl_allocator_t allocator = rcl_get_default_allocator();
      rcl_ret_t rc;

      // create init_options
      rc = rcl_init_options_init(&init_options, allocator);
      if (rc != RCL_RET_OK) {
        printf("Error rcl_init_options_init.\n");
        return false;
      }

      // create context
      rc = rcl_init(0, nullptr, &init_options, &context);
      if (rc != RCL_RET_OK) {
        printf("Error in rcl_init.\n");
        return false;
      }

      // create rcl_node
      rcl_node_t my_node = rcl_get_zero_initialized_node();
      rcl_node_options_t node_ops = rcl_node_get_default_options();
      rc =
          rcl_node_init(&my_node, "node_0", "custom_node", &context, &node_ops);
      if (rc != RCL_RET_OK) {
        printf("Error in rcl_node_init\n");
        return false;
      }

      const char *topic_name = "my_string";

      const rosidl_message_type_support_t *my_type_support =
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String);

      rcl_publisher_options_t pub_options = rcl_publisher_get_default_options();

      // Initialize Publisher
      rc = rcl_publisher_init(&state.my_pub, &my_node, my_type_support,
                              topic_name, &pub_options);
      if (RCL_RET_OK != rc) {
        printf("Error in rcl_publisher_init %s.\n", topic_name);
        return false;
      }

      std::string topic_name_string_pcl = db.inputs.topicName();
      const char *topic_name_pcl = topic_name_string_pcl.c_str();

      const rosidl_message_type_support_t *my_type_support_pcl =
          ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, PointCloud2);

      rcl_publisher_options_t pub_options_pcl =
          rcl_publisher_get_default_options();

      // Initialize Publisher
      rc = rcl_publisher_init(&state.my_pub_pcl, &my_node, my_type_support_pcl,
                              topic_name_pcl, &pub_options_pcl);
      if (RCL_RET_OK != rc) {
        printf("Error in rcl_publisher_init %s.\n", topic_name);
        return false;
      }
      // Node, publisher was successfully created
      state.pub_created = true;

      return true;
    }

    std_msgs__msg__String *ros_msg = std_msgs__msg__String__create();

    // Get the string to be published
    std::string msg = db.inputs.frameId();

    // Assign our ros message the data from this string
    rosidl_runtime_c__String__assign(&ros_msg->data, msg.c_str());

    rcl_ret_t rc;
    rc = rcl_publish(&state.my_pub, ros_msg, NULL);
    if (rc != RCL_RET_OK) {
      // RCL_RET_PUBLISHER_INVALID is returned initially and then the message
      // gets published
      return false;
    }

    // Destroy the ROS message published to release the memory it used
    std_msgs__msg__String__destroy(ros_msg);
    /*
    sensor_msgs__msg__PointCloud2 *ros_msg_pcl =
        sensor_msgs__msg__PointCloud2__create();

    // Assign our ros message the data from this string
    ros_msg_pcl->header.stamp.sec = 0;
    ros_msg_pcl->header.stamp.nanosec = 0;
    std::string frameId = db.inputs.frameId();
    rosidl_runtime_c__String__assign(&ros_msg_pcl->header.frame_id,
                                     frameId.c_str());

    ros_msg_pcl->height = 1;
    ros_msg_pcl->width = 2; // TODO: Change to size of point cloud

    ros_msg_pcl->fields.size = 4;
    ros_msg_pcl->fields.data =
        static_cast<sensor_msgs__msg__PointField *>(malloc(
            ros_msg_pcl->fields.size * sizeof(sensor_msgs__msg__PointField)));

    const char *xCString = "x";
    const char *yCString = "y";
    const char *zCString = "z";
    const char *iCString = "intensity";
    rosidl_runtime_c__String__assign(&ros_msg_pcl->fields.data[0].name,
                                     xCString);
    rosidl_runtime_c__String__assign(&ros_msg_pcl->fields.data[1].name,
                                     yCString);
    rosidl_runtime_c__String__assign(&ros_msg_pcl->fields.data[2].name,
                                     zCString);
    rosidl_runtime_c__String__assign(&ros_msg_pcl->fields.data[3].name,
                                     iCString);

    int offset = 0;
    // All offsets are *4, as all field data types are float32
    for (size_t d = 0; d < ros_msg_pcl->fields.size; ++d, offset += 4) {
      ros_msg_pcl->fields.data[d].offset = offset;
      ros_msg_pcl->fields.data[d].datatype =
          sensor_msgs__msg__PointField__FLOAT32;
      ros_msg_pcl->fields.data[d].count = 1;
    }

    ros_msg_pcl->is_bigendian = false;
    ros_msg_pcl->point_step = offset;
    ros_msg_pcl->row_step = ros_msg_pcl->point_step * ros_msg_pcl->width;

    ros_msg_pcl->data.size = ros_msg_pcl->row_step * ros_msg_pcl->height;
    ros_msg_pcl->data.data = static_cast<uint8_t *>(
        malloc(ros_msg_pcl->data.size * sizeof(uint8_t)));

    ros_msg_pcl->fields.data =
        static_cast<sensor_msgs__msg__PointField *>(malloc(
            ros_msg_pcl->fields.size * sizeof(sensor_msgs__msg__PointField)));

    ros_msg_pcl->is_dense = false;
    for (size_t i = 0; i < ros_msg_pcl->data.size; ++i) {
      ros_msg_pcl->data.data[i] =
          i % 256; // Just an example, replace with your actual data
    }
    // rcl_ret_t rc;
    rc = rcl_publish(&state.my_pub_pcl, ros_msg_pcl, NULL);
    if (rc != RCL_RET_OK) {
      // RCL_RET_PUBLISHER_INVALID is returned initially and then the message
      // gets published
      return false;
    }

    // Destroy the ROS message published to release the memory it used
    free(ros_msg_pcl->fields.data);
    free(ros_msg_pcl->data.data);
    sensor_msgs__msg__PointCloud2__destroy(ros_msg_pcl);
     */
    // Returning true tells Omnigraph that the compute was successful and
    // the output value is now valid.
    return true;
  }

private:
  rcl_publisher_t my_pub;
  rcl_publisher_t my_pub_pcl;
  bool pub_created{false};
};

// This macro provides the information necessary to OmniGraph that lets it
// automatically register and deregister your node type definition.
REGISTER_OGN_NODE()

} // namespace omnigraph_node
} // namespace ros2_cpp
} // namespace custom
