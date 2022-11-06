#include <factor_graph_visual.hpp>

#include <iostream>
#include <rviz_lines.hpp>

namespace rviz_factor_graph_plugins {

struct PoseNode {
  PoseNode(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node) {
    node = parent_node->createChildSceneNode();
    axes.reset(new rviz_rendering::Axes(scene_manager, node, 0.5f, 0.05f));
  }
  ~PoseNode() {}

  void setPointCloud(const sensor_msgs::msg::PointCloud2& points_msg) {
    int x_offset = -1;
    int y_offset = -1;
    int z_offset = -1;

    for (const auto& field : points_msg.fields) {
      if (field.name == "x") {
        x_offset = field.offset;
      } else if (field.name == "y") {
        y_offset = field.offset;
      } else if (field.name == "z") {
        z_offset = field.offset;
      }

      if (field.datatype != 7) {
        std::cerr << "warning: only float32 point coordinates are supported!!" << std::endl;
      }
    }

    const int num_points = points_msg.width * points_msg.height;
    std::vector<rviz_rendering::PointCloud::Point> points(num_points);
    for (int i = 0; i < num_points; i++) {
      points[i].position.x = *reinterpret_cast<const float*>(points_msg.data.data() + points_msg.point_step * i + x_offset);
      points[i].position.y = *reinterpret_cast<const float*>(points_msg.data.data() + points_msg.point_step * i + y_offset);
      points[i].position.z = *reinterpret_cast<const float*>(points_msg.data.data() + points_msg.point_step * i + z_offset);
    }

    this->points = std::make_shared<rviz_rendering::PointCloud>();
    this->points->setName("points");
    this->points->addPoints(points.begin(), points.end());
    this->points->setRenderMode(rviz_rendering::PointCloud::RenderMode::RM_POINTS);

    node->attachObject(this->points.get());
  }

  void setPose(const Ogre::Vector3& position, const Ogre::Quaternion& orientation) {
    node->setPosition(position);
    node->setOrientation(orientation);
  }
  Ogre::Vector3 getPosition() const { return node->getPosition(); }
  Ogre::Quaternion getOrientation() const { return node->getOrientation(); }

  void setAxesShape(float length, float radius) {  //
    axes->set(length, radius);
  }

  Ogre::SceneNode* node;

  std::shared_ptr<rviz_rendering::Axes> axes;
  std::shared_ptr<rviz_rendering::PointCloud> points;
};

FactorGraphVisual::FactorGraphVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node) {
  this->scene_manager_ = scene_manager;
  this->frame_node_ = parent_node->createChildSceneNode();

  axes_length = 0.5f;
  axes_radius = 0.02f;

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
  this->pose_nodes.clear();

  this->trajectory_lines.reset(new Lines(scene_manager_, frame_node_));
  this->trajectory_lines->setColor(1.0f, 0.0f, 0.0f, 1.0f);

  this->factor_lines.reset(new Lines(scene_manager_, frame_node_));
  this->factor_lines->setColor(0.0f, 1.0f, 0.0f, 0.2f);
}

void FactorGraphVisual::update() {
  if (!get_point_cloud->service_is_ready() || !last_graph_msg || last_graph_msg->poses.empty()) {
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
        node->second->setPointCloud(response->points);
      }
    }

    std::cout << "response:" << response->success << " " << response->key << " " << response->points.width * response->points.height << std::endl;
  }

  const int count = 5;
  while (get_point_cloud_results.size() < count) {
    const auto& poses = last_graph_msg->poses;
    const auto& pose = poses[(loading_counter++) % poses.size()];

    auto req = std::make_shared<GetPointCloud::Request>();
    req->key = pose.key;

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

void FactorGraphVisual::setGetPointCloudService(rclcpp::Client<GetPointCloud>::SharedPtr service) {
  get_point_cloud = service;
}

}  // namespace rviz_factor_graph_plugins