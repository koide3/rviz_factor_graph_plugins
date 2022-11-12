#include <factor_graph_visual.hpp>

#include <iostream>
#include <pose_node.hpp>
#include <rviz_lines.hpp>

namespace rviz_factor_graph_plugins {

FactorGraphVisual::FactorGraphVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node) {
  this->scene_manager_ = scene_manager;
  this->frame_node_ = parent_node->createChildSceneNode();
  factor_lines.reset(new Lines(scene_manager, this->frame_node_));

  max_requests = 5;
  max_load_count = 15;

  show_axes = true;
  axes_length = 0.5f;
  axes_radius = 0.02f;

  show_points = true;
  point_style = rviz_rendering::PointCloud::RM_FLAT_SQUARES;
  color_settings.reset(new PointColorSettings);

  loading_counter = 0;
}

FactorGraphVisual::~FactorGraphVisual() {
  frame_node_->detachAllObjects();
  scene_manager_->destroySceneNode(frame_node_);
}

void FactorGraphVisual::reset() {
  last_graph_msg.reset();
  pose_nodes.clear();
  factor_lines->clear();
}

void FactorGraphVisual::update() {
  if (!get_point_cloud->wait_for_service(std::chrono::nanoseconds(1)) || !last_graph_msg || last_graph_msg->poses.empty()) {
    return;
  }

  // Get asynchronously fetched points
  while (!get_point_cloud_results.empty()) {
    auto& result = get_point_cloud_results.front().first;

    if (result.wait_for(std::chrono::nanoseconds(1)) != std::future_status::ready) {
      break;
    }

    const auto response = result.get();
    get_point_cloud_results.pop_front();

    const char chr = (response->key >> 56);
    const size_t index = ((response->key << 8) >> 8);

    if (response->success) {
      auto node = pose_nodes.find(response->key);
      if (node != pose_nodes.end()) {
        node->second->setPointCloud(response->points, color_settings);
        node->second->setPointStyle(point_size, point_alpha, point_style);
      } else {
        RCLCPP_WARN_STREAM(rclcpp::get_logger("rviz_factor_graph_plugins"), "Node for the fetched points is not found key=" << response->key << " symbol=" << chr << index);
      }
    } else {
      RCLCPP_WARN_STREAM(rclcpp::get_logger("rviz_factor_graph_plugins"), "Failed to retrieve points for key=" << response->key << " symbol=" << chr << index);
    }
  }

  // Issue asynchronous points fetching calls
  for (int i = 0; i < max_load_count && get_point_cloud_results.size() < max_requests; i++) {
    auto req = std::make_shared<GetPointCloud::Request>();

    // Load points for newly created points
    if (!load_priority_queue.empty()) {
      req->key = load_priority_queue.front();
      load_priority_queue.pop_front();
    }
    // Load points for existing nodes without points if the priority queue is empty
    else {
      const auto& poses = last_graph_msg->poses;
      const auto& pose = poses[(loading_counter++) % poses.size()];

      if (pose.type != factor_graph_interfaces::msg::PoseWithID::POINTS) {
        continue;
      }

      req->key = pose.key;
    }

    auto found = pose_nodes.find(req->key);
    if (found == pose_nodes.end()) {
      continue;
    }

    if (found->second->points && !found->second->recoloringRequired(*color_settings)) {
      // Avoid re-loading because the node already has points
      continue;
    }

    get_point_cloud_results.emplace_back(get_point_cloud->async_send_request(req), get_point_cloud);
  }
}

void FactorGraphVisual::setVisibility(bool show_factors, bool show_axes, bool show_points) {
  factor_lines->setVisible(show_factors);

  this->show_axes = show_axes;
  this->show_points = show_points;

  for (auto& node : pose_nodes) {
    node.second->setVisibility(show_axes, show_points);
  }
}

void FactorGraphVisual::setFactorColor(const Ogre::ColourValue& factor_color) {
  factor_lines->setColor(factor_color);
}

void FactorGraphVisual::setPose(const Ogre::Vector3& position, const Ogre::Quaternion& orientation) {
  frame_node_->setPosition(position);
  frame_node_->setOrientation(orientation);
}

void FactorGraphVisual::setMessage(const FactorGraph::ConstSharedPtr& graph_msg) {
  last_graph_msg = graph_msg;

  for (int i = 0; i < graph_msg->poses.size(); i++) {
    const std::uint64_t key = graph_msg->poses[i].key;
    const bool has_points = graph_msg->poses[i].type == factor_graph_interfaces::msg::PoseWithID::POINTS;

    const auto& trans = graph_msg->poses[i].pose.position;
    const auto& quat = graph_msg->poses[i].pose.orientation;

    const Ogre::Vector3 pos(trans.x, trans.y, trans.z);
    const Ogre::Quaternion ori(quat.w, quat.x, quat.y, quat.z);

    auto node = pose_nodes.find(key);
    if (node == pose_nodes.end()) {
      node = pose_nodes.emplace_hint(node, key, new PoseNode(scene_manager_, frame_node_));
      node->second->setAxesShape(axes_length, axes_radius);
      node->second->setVisibility(show_axes, show_points);

      if (has_points) {
        load_priority_queue.emplace_back(key);
      }
    }
    node->second->setPose(pos, ori);
  }

  // TODO: use indexed lines for better performance
  std::vector<Ogre::Vector3> factor_points;
  factor_points.reserve(graph_msg->binary_factors.size());

  for (const auto& factor : graph_msg->binary_factors) {
    const auto node1 = pose_nodes.find(factor.keys[0]);
    const auto node2 = pose_nodes.find(factor.keys[1]);

    if (node1 == pose_nodes.end() || node2 == pose_nodes.end()) {
      continue;
    }

    factor_points.emplace_back(node1->second->getPosition());
    factor_points.emplace_back(node2->second->getPosition());
  }
  this->factor_lines->setPoints(factor_points, false);
}

void FactorGraphVisual::setAxesShape(float length, float radius) {
  axes_length = length;
  axes_radius = radius;

  for (auto& node : pose_nodes) {
    node.second->setAxesShape(axes_length, axes_radius);
  }
}

void FactorGraphVisual::setPointStyle(float size, float alpha, rviz_rendering::PointCloud::RenderMode mode) {
  point_size = size;
  point_alpha = alpha;
  point_style = mode;

  for (auto& node : pose_nodes) {
    node.second->setPointStyle(point_size, point_alpha, point_style);
  }
}

void FactorGraphVisual::setColorSettings(const std::shared_ptr<PointColorSettings>& color_settings) {
  for (auto& node : pose_nodes) {
    const auto& settings = node.second->color_settings;

    bool needs_recoloring = false;
    needs_recoloring |= (settings.mode != color_settings->mode);
    needs_recoloring |= (settings.axis != color_settings->axis);
    needs_recoloring |= (settings.colormap != color_settings->colormap);
    needs_recoloring |= std::abs(settings.color.r - color_settings->color.r) > 1e-3;
    needs_recoloring |= std::abs(settings.color.g - color_settings->color.g) > 1e-3;
    needs_recoloring |= std::abs(settings.color.b - color_settings->color.b) > 1e-3;

    if (needs_recoloring) {
      node.second->points.reset();
    }
  }

  this->color_settings = color_settings;
}

void FactorGraphVisual::setPointsLoadingParams(int max_load_count, int max_requests) {
  this->max_requests = max_requests;
  this->max_load_count = max_load_count;
}

void FactorGraphVisual::setGetPointCloudService(std::shared_ptr<rclcpp::Node> node, rclcpp::Client<GetPointCloud>::SharedPtr service) {
  get_point_cloud = service;
}

}  // namespace rviz_factor_graph_plugins