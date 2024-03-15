// Copyright (c) 2022, NVIDIA CORPORATION. All rights reserved.
//
// NVIDIA CORPORATION and its licensors retain all intellectual property
// and proprietary rights in and to this software, related documentation
// and any modifications thereto.  Any use, reproduction, disclosure or
// distribution of this software and related documentation without an express
// license agreement from NVIDIA CORPORATION is strictly prohibited.
//
#include <ROS2RTXLiDARPublishPointCloudNodeDatabase.h>

// ROS messages includes
#include "rosidl_runtime_c/primitives_sequence_functions.h"
#include "rosidl_runtime_c/string_functions.h"
#include "sensor_msgs/msg/detail/point_field__functions.h"
#include "sensor_msgs/msg/point_cloud2.h"
#include "std_msgs/msg/header.h"
#include "std_msgs/msg/string.h"
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>

//  ROS includes for creating nodes, publishers etc.
#include "rcl/error_handling.h"
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
      // Initialization for ros publisher
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
      std::string node_namespace_string_pcl = db.inputs.nodeNamespace();
      const char *node_namespace_pcl = node_namespace_string_pcl.c_str();
      rc = rcl_node_init(&my_node, "ROS2RTXLiDARPublishPointCloudNode_0",
                         node_namespace_pcl, &context, &node_ops);
      if (rc != RCL_RET_OK) {
        printf("Error in rcl_node_init\n");
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
        printf("Error in rcl_publisher_init %s.\n", topic_name_pcl);
        return false;
      }
      // Node, publisher was successfully created
      state.pub_created = true;

      return true;
    }
    if (!db.inputs.enabled())
      return true;

    // const omni::graph::core::ogn::const_array <float>
    auto input_x = db.inputs.x();
    auto input_y = db.inputs.y();
    auto input_z = db.inputs.z();
    auto input_intensity = db.inputs.intensity();

    uint32_t width = static_cast<uint32_t>(
        std::min({input_x.size(), input_y.size(), input_z.size(),
                  input_intensity.size()}));

    sensor_msgs__msg__PointCloud2 *ros_msg_pcl =
        sensor_msgs__msg__PointCloud2__create();

    double timestamp = db.inputs.timestamp();
    auto sec = std::floor(timestamp);
    auto nsec = 1e9 * (timestamp - sec);
    ros_msg_pcl->header.stamp.sec = static_cast<uint32_t>(sec);
    ros_msg_pcl->header.stamp.nanosec = static_cast<uint32_t>(nsec);

    std::string frameId = db.inputs.frameId();
    rosidl_runtime_c__String__assign(&ros_msg_pcl->header.frame_id,
                                     frameId.c_str());

    ros_msg_pcl->height = 1;
    ros_msg_pcl->width = width;

    sensor_msgs__msg__PointField__Sequence__init(&(ros_msg_pcl->fields), 4);

    std::string xString = "x";
    std::string yString = "y";
    std::string zString = "z";
    std::string iString = "intensity";
    rosidl_runtime_c__String__assign(&(ros_msg_pcl->fields.data[0].name),
                                     xString.c_str());
    rosidl_runtime_c__String__assign(&(ros_msg_pcl->fields.data[1].name),
                                     yString.c_str());
    rosidl_runtime_c__String__assign(&(ros_msg_pcl->fields.data[2].name),
                                     zString.c_str());
    rosidl_runtime_c__String__assign(&(ros_msg_pcl->fields.data[3].name),
                                     iString.c_str());
    int offset = 0;
    // All offsets are 4, as all field data types are float32
    for (size_t d = 0; d < ros_msg_pcl->fields.size; ++d, offset += 4) {
      ros_msg_pcl->fields.data[d].offset = offset;
      ros_msg_pcl->fields.data[d].datatype =
          sensor_msgs__msg__PointField__FLOAT32;
      ros_msg_pcl->fields.data[d].count = 1;
    }

    ros_msg_pcl->is_bigendian = false;
    ros_msg_pcl->point_step = offset;
    ros_msg_pcl->row_step = ros_msg_pcl->point_step * ros_msg_pcl->width;
    rosidl_runtime_c__uint8__Sequence__init(
        &(ros_msg_pcl->data), ros_msg_pcl->row_step * ros_msg_pcl->height);

    // Type punning using union have undefined behavior with undefined float
    // size
    assert(CHAR_BIT * sizeof(float) == 32);
    assert(CHAR_BIT == 8);
    // Type punning through union
    union {
      float floatValue;
      uint8_t uint8Value[4];
    } floatToUint8Converter;

    for (size_t point_i = 0; point_i < ros_msg_pcl->width; point_i++) {
      for (size_t field_i = 0; field_i < ros_msg_pcl->fields.size; field_i++) {
        for (size_t byte_i = 0; byte_i < 4; byte_i) {
          switch (field_i % ros_msg_pcl->field.size) {
          case 0:
            floatToUint8Converter.floatValue = input_x[point_i];
            break;
          case 1:
            floatToUint8Converter.floatValue = input_y[point_i];
            break;
          case 2:
            floatToUint8Converter.floatValue = input_z[point_i];
            break;
          case 3:
            floatToUint8Converter.floatValue = input_intensity[point_i];
            break;
          }
          ros_msg_pcl->data.data[byte_i + 4 * field_i +
                                 4 * ros_msg_pcl->fields.size * point_i] =
              floatToUint8Converter.uint8Value[byte_i];
        }
      }
    }
    ros_msg_pcl->is_dense = false;

    // rcl_ret_t rc;
    rcl_ret_t rc;
    rc = rcl_publish(&state.my_pub_pcl, ros_msg_pcl, NULL);
    if (rc != RCL_RET_OK) {
      // RCL_RET_PUBLISHER_INVALID is returnedinitially initially and then the
      // message gets published
      return false;
    }

    // Destroy the ROS message published to release the memory it used
    sensor_msgs__msg__PointCloud2__destroy(ros_msg_pcl);

    // Returning true tells Omnigraph that the
    // compute was successful and the output
    // value is now valid.
    return true;
  }

private:
  rcl_publisher_t my_pub_pcl;
  bool pub_created{false};
};

// This macro provides the information necessary to OmniGraph that lets it
// automatically register and deregister your node type definition.
REGISTER_OGN_NODE()

} // namespace omnigraph_node
} // namespace ros2_cpp
} // namespace custom
