#include <factor_graph_visual.hpp>

#include <iostream>
#include <pose_node.hpp>
#include <rviz_lines.hpp>

namespace rviz_factor_graph_plugins {

FactorGraphVisual::FactorGraphVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node) {
  this->scene_manager_ = scene_manager;
  this->frame_node_ = parent_node->createChildSceneNode();

  axes_length = 0.5f;
  axes_radius = 0.02f;

  point_style = rviz_rendering::PointCloud::RM_FLAT_SQUARES;
  color_settings.reset(new PointColorSettings);

  this->trajectory_lines.reset(new Lines(scene_manager, this->frame_node_));
  this->trajectory_lines->setColor(1.0f, 0.0f, 0.0f, 1.0f);

  this->factor_lines.reset(new Lines(scene_manager, this->frame_node_));
  this->factor_lines->setColor(0.0f, 1.0f, 0.0f, 0.2f);

  loading_counter = 0;
}

FactorGraphVisual::~FactorGraphVisual() {
  frame_node_->detachAllObjects();
  scene_manager_->destroySceneNode(frame_node_);
}

void FactorGraphVisual::reset() {
  this->last_graph_msg.reset();
  this->pose_nodes.clear();

  this->trajectory_lines.reset(new Lines(scene_manager_, frame_node_));
  this->trajectory_lines->setColor(1.0f, 0.0f, 0.0f, 1.0f);

  this->factor_lines.reset(new Lines(scene_manager_, frame_node_));
  this->factor_lines->setColor(0.0f, 1.0f, 0.0f, 0.2f);
}

void FactorGraphVisual::update() {
  if (!get_point_cloud->wait_for_service(std::chrono::nanoseconds(1)) || !last_graph_msg || last_graph_msg->poses.empty()) {
    return;
  }

  while (!get_point_cloud_results.empty()) {
    auto& result = get_point_cloud_results.front();

    if (result.wait_for(std::chrono::nanoseconds(1)) != std::future_status::ready) {
      break;
    }

    const auto response = result.get();
    get_point_cloud_results.pop_front();

    if (response->success) {
      auto node = pose_nodes.find(response->key);
      if (node != pose_nodes.end()) {
        node->second->setPointCloud(response->points, color_settings);
        node->second->setPointStyle(point_size, point_alpha, point_style);
      }
    }
  }

  const int count = 5;
  for (int i = 0; i < count && get_point_cloud_results.size() < count; i++) {
    auto req = std::make_shared<GetPointCloud::Request>();

    if (!load_priority_queue.empty()) {
      req->key = load_priority_queue.front();
      load_priority_queue.pop_front();
    } else {
      const auto& poses = last_graph_msg->poses;
      req->key = poses[(loading_counter++) % poses.size()].key;
    }

    auto found = pose_nodes.find(req->key);
    if (found == pose_nodes.end()) {
      continue;
    }

    if (found->second->points && !found->second->recoloringRequired(*color_settings)) {
      continue;
    }

    get_point_cloud_results.emplace_back(get_point_cloud->async_send_request(req));
  }
}

void FactorGraphVisual::setPose(const Ogre::Vector3& position, const Ogre::Quaternion& orientation) {
  frame_node_->setPosition(position);
  frame_node_->setOrientation(orientation);
}

void FactorGraphVisual::setMessage(const FactorGraph::ConstSharedPtr& graph_msg) {
  last_graph_msg = graph_msg;

  std::vector<Ogre::Vector3> traj_positions(graph_msg->poses.size());

  for (int i = 0; i < graph_msg->poses.size(); i++) {
    const std::uint64_t key = graph_msg->poses[i].key;
    const auto& trans = graph_msg->poses[i].pose.position;
    const auto& quat = graph_msg->poses[i].pose.orientation;

    const Ogre::Vector3 pos(trans.x, trans.y, trans.z);
    const Ogre::Quaternion ori(quat.w, quat.x, quat.y, quat.z);

    auto node = pose_nodes.find(key);
    if (node == pose_nodes.end()) {
      node = pose_nodes.emplace_hint(node, key, new PoseNode(scene_manager_, frame_node_));
      node->second->setAxesShape(axes_length, axes_radius);
      load_priority_queue.emplace_back(key);
    }
    node->second->setPose(pos, ori);
    traj_positions[i] = pos;
  }

  trajectory_lines->setPoints(traj_positions, true);

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
    needs_recoloring |= std::abs(settings.color.r - color_settings->color.r) > 1e-3;
    needs_recoloring |= std::abs(settings.color.g - color_settings->color.g) > 1e-3;
    needs_recoloring |= std::abs(settings.color.b - color_settings->color.b) > 1e-3;

    if (needs_recoloring) {
      node.second->points.reset();
    }
  }

  this->color_settings = color_settings;
}

void FactorGraphVisual::setGetPointCloudService(std::shared_ptr<rclcpp::Node> node, rclcpp::Client<GetPointCloud>::SharedPtr service) {
  // poor way to avoid accessing disposed service calls
  if (!get_point_cloud_results.empty()) {
    get_point_cloud_results_disposed.insert(get_point_cloud_results_disposed.end(), get_point_cloud_results.begin(), get_point_cloud_results.end());
    get_point_cloud_results.clear();
  }

  get_point_cloud = service;
}

}  // namespace rviz_factor_graph_plugins