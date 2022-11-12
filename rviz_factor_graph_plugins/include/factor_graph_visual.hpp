#pragma once

#include <deque>
#include <unordered_map>
#include <rclcpp/rclcpp.hpp>

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

#include <rviz_rendering/objects/axes.hpp>
#include <rviz_rendering/objects/point_cloud.hpp>

#include <factor_graph_interfaces/msg/factor_graph.hpp>
#include <factor_graph_interfaces/srv/get_point_cloud.hpp>

#include <point_color_settings.hpp>

namespace rviz_factor_graph_plugins {

class Lines;
struct PoseNode;

class FactorGraphVisual {
public:
  using FactorGraph = factor_graph_interfaces::msg::FactorGraph;
  using GetPointCloud = factor_graph_interfaces::srv::GetPointCloud;

  FactorGraphVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node);
  ~FactorGraphVisual();

  void reset();
  void update();

  void setVisibility(bool show_factors, bool show_axes, bool show_points);
  void setFactorColor(const Ogre::ColourValue& factor_color);

  void setPose(const Ogre::Vector3& position, const Ogre::Quaternion& orientation);
  void setMessage(const FactorGraph::ConstSharedPtr& graph_msg);

  void setAxesShape(float length, float radius);
  void setPointStyle(float size, float alpha, rviz_rendering::PointCloud::RenderMode mode);
  void setColorSettings(const std::shared_ptr<PointColorSettings>& color_settings);

  void setPointsLoadingParams(int max_load_count, int max_requests);
  void setGetPointCloudService(std::shared_ptr<rclcpp::Node> node, rclcpp::Client<GetPointCloud>::SharedPtr service);

private:
  std::unordered_map<std::uint64_t, std::shared_ptr<PoseNode>> pose_nodes;
  std::shared_ptr<Lines> factor_lines;

  bool show_axes;
  float axes_length;
  float axes_radius;

  bool show_points;
  float point_size;
  float point_alpha;
  rviz_rendering::PointCloud::RenderMode point_style;

  std::shared_ptr<PointColorSettings> color_settings;

  Ogre::SceneNode* frame_node_;
  Ogre::SceneManager* scene_manager_;

  rclcpp::Client<GetPointCloud>::SharedPtr get_point_cloud;

  int max_requests;
  int max_load_count;

  std::uint64_t loading_counter;
  FactorGraph::ConstSharedPtr last_graph_msg;
  std::deque<std::uint64_t> load_priority_queue;
  std::deque<std::pair<std::shared_future<GetPointCloud::Response::SharedPtr>, rclcpp::Client<GetPointCloud>::SharedPtr>> get_point_cloud_results;
};
}  // namespace rviz_factor_graph_plugins