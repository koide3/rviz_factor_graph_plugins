cmake_minimum_required(VERSION 3.5)
project(rviz_factor_graph_plugins)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

set(CMAKE_AUTOMOC ON)

find_package(Qt5 REQUIRED COMPONENTS Widgets)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

qt5_wrap_cpp(rviz_factor_graph_plugins_moc_files
  include/factor_graph_display.hpp
)
ament_auto_add_library(rviz_factor_graph_plugins SHARED
  src/rviz_lines.cpp
  src/pose_node.cpp
  src/factor_graph_visual.cpp
  src/factor_graph_display.cpp
  src/rviz_factor_graph_plugins.cpp
  ${rviz_factor_graph_plugins_moc_files}
)
target_include_directories(rviz_factor_graph_plugins PUBLIC
  include
  ${Qt5Widgets_INCLUDE_DIRS}
)

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

ament_auto_package()
