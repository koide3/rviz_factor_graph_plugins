cmake_minimum_required(VERSION 3.5)
project(factor_graph_interfaces)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(factor_graph_interfaces
  "msg/PoseWithID.msg"
  "msg/PointWithID.msg"
  "msg/PriorFactor.msg"
  "msg/BetweenFactor.msg"
  "msg/FactorGraph.msg"
  "srv/GetPointCloud.srv"
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs
)

ament_export_include_directories(include)
ament_export_dependencies(rosidl_default_runtime std_msgs geometry_msgs)

ament_auto_package()
