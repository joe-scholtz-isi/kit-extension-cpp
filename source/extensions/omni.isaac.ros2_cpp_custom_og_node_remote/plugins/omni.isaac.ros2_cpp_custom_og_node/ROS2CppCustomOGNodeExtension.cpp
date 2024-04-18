// Copyright (c) 2022, NVIDIA CORPORATION. All rights reserved.
//
// NVIDIA CORPORATION and its licensors retain all intellectual property
// and proprietary rights in and to this software, related documentation
// and any modifications thereto.  Any use, reproduction, disclosure or
// distribution of this software and related documentation without an express
// license agreement from NVIDIA CORPORATION is strictly prohibited.
//

#define CARB_EXPORTS

#include <carb/PluginUtils.h>

#include <omni/ext/IExt.h>
#include <omni/graph/core/IGraphRegistry.h>
#include <omni/graph/core/ogn/Database.h>
#include <omni/graph/core/ogn/Registration.h>

// Standard plugin definitions required by Carbonite.
const struct carb::PluginImplDesc pluginImplDesc = {
    "custom.ros2_cpp.omnigraph_node.plugin",
    "An example for Custom ROS2 OGN extension", "NVIDIA",
    carb::PluginHotReload::eEnabled, "dev"};

// These interface dependencies are required by all OmniGraph node types
CARB_PLUGIN_IMPL_DEPS(omni::graph::core::IGraphRegistry, omni::fabric::IPath,
                      omni::fabric::IToken)

// This macro sets up the information required to register your node type
// definitions with OmniGraph
DECLARE_OGN_NODES()

namespace omni {
namespace isaac {
namespace ros2_cpp_custom_og_node {

class ROS2CppCustomOGNodeExtension : public omni::ext::IExt {
public:
  void onStartup(const char *extId) override {
    printf("Custom ROS2 OGN Nodes extension startup..... (ext_id: %s).\n",
           extId);
    // This macro walks the list of pending node type definitions and registers
    // them with OmniGraph
    INITIALIZE_OGN_NODES()
  }

  void onShutdown() override {
    printf("Custom ROS2 OGN Nodes extension shutdown\n");
    // This macro walks the list of registered node type definitions and
    // deregisters all of them. This is required for hot reload to work.
    RELEASE_OGN_NODES()
  }

private:
};

} // namespace ros2_cpp_custom_og_node
} // namespace isaac
} // namespace omni

CARB_PLUGIN_IMPL(
    pluginImplDesc,
    omni::isaac::ros2_cpp_custom_og_node::ROS2CppCustomOGNodeExtension)

void fillInterface(
    omni::isaac::ros2_cpp_custom_og_node::ROS2CppCustomOGNodeExtension &iface) {
}
