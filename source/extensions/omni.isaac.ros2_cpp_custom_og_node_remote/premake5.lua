-- Setup the basic extension information.
local ext = get_current_extension_info()
project_ext(ext)

-- --------------------------------------------------------------------------------------------------------------
-- Helper variable containing standard configuration information for projects containing OGN files.
local ogn = get_ogn_project_information(ext, "omni/isaac/ros2_cpp_custom_og_node")


-- --------------------------------------------------------------------------------------------------------------------
-- Put this project into the "omnigraph" IDE group
ext.group = "omnigraph"

-- --------------------------------------------------------------------------------------------------------------
-- Link folders that should be packaged with the extension.
repo_build.prebuild_link {
  { "data", ext.target_dir .. "/data" },
  { "docs", ext.target_dir .. "/docs" },
}

-- --------------------------------------------------------------------------------------------------------------
-- Copy the __init__.py to allow building of a non-linked ogn/ import directory.
-- In a mixed extension this would be part of a separate Python-based project but since here it is just the one
-- file it can be copied directly with no build dependencies.
repo_build.prebuild_copy {
  { "omni/isaac/ros2_cpp_custom_og_node/__init__.py", ogn.python_target_path }
}

-- --------------------------------------------------------------------------------------------------------------
-- Breaking this out as a separate project ensures the .ogn files are processed before their results are needed.
project_ext_ogn(ext, ogn, { toc = "docs/Overview.md" })

-- --------------------------------------------------------------------------------------------------------------
-- Build the C++ plugin that will be loaded by the extension.
project_ext_plugin(ext, ogn.plugin_project)
-- It is important that you add all subdirectories containing C++ code to this project
add_files("source", "plugins/" .. ogn.module)
add_files("nodes", "plugins/nodes")
add_files("python/nodes", "python/nodes")
add_files("config", "config")
add_files("docs", "docs")
add_files("data", "data")

-- Add the standard dependencies all OGN projects have; includes, libraries to link, and required compiler flags
-- add_ogn_dependencies(ogn)
add_ogn_dependencies(ogn, { "python/nodes" })

includedirs {
  -- System level ROS includes
  "%{root}/_build/target-deps/system_ros/include/std_msgs",

  "%{root}/_build/target-deps/system_ros/include/sensor_msgs",

  "%{root}/_build/target-deps/system_ros/include/geometry_msgs",

  "%{root}/_build/target-deps/system_ros/include/builtin_interfaces",

  "%{root}/_build/target-deps/system_ros/include/rosidl_runtime_c",

  "%{root}/_build/target-deps/system_ros/include/rosidl_runtime_cpp",

  "%{root}/_build/target-deps/system_ros/include/rosidl_typesupport_interface",

  "%{root}/_build/target-deps/system_ros/include/rcl",

  "%{root}/_build/target-deps/system_ros/include/rcutils",

  "%{root}/_build/target-deps/system_ros/include/rmw",

  "%{root}/_build/target-deps/system_ros/include/rcl_yaml_param_parser",

}

libdirs {
  -- System level ROS libraries
  "%{root}/_build/target-deps/system_ros/lib",
}

links {
  --  Minimal ROS 2 C API libs needed for your nodes to work
  "rosidl_runtime_c", "rcutils", "rcl", "rmw",

  -- For the simple string message, add the deps
  "std_msgs__rosidl_typesupport_c", "std_msgs__rosidl_generator_c",
  "sensor_msgs__rosidl_typesupport_c", "sensor_msgs__rosidl_generator_c",
}


filter { "system:linux" }
includedirs { "%{target_deps}/python/include/python3.10" }
linkoptions { "-Wl,--export-dynamic" }
