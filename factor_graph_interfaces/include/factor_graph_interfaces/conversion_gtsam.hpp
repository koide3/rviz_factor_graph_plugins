#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <factor_graph_interfaces/msg/factor_graph.hpp>

namespace factor_graph_interfaces {

factor_graph_interfaces::msg::FactorGraph::SharedPtr convert_to_msg(const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& values) {
  using namespace msg;
  auto graph_msg = std::make_shared<FactorGraph>();

  std::cout << graph.size() << " factors   " << values.size() << " values" << std::endl;

  std::unordered_set<std::uint64_t> keys;
  for (const auto& value : values) {
    const auto pose_value = dynamic_cast<const gtsam::GenericValue<gtsam::Pose3>*>(&value.value);
    if (pose_value) {
      const gtsam::Pose3& pose = pose_value->value();

      const Eigen::Vector3d trans = pose.translation();
      const Eigen::Quaterniond quat(pose.rotation().matrix());

      PoseWithID msg;
      msg.type = PoseWithID::NONE;
      msg.key = value.key;

      msg.pose.position.x = trans.x();
      msg.pose.position.y = trans.y();
      msg.pose.position.z = trans.z();

      msg.pose.orientation.x = quat.x();
      msg.pose.orientation.y = quat.y();
      msg.pose.orientation.z = quat.z();
      msg.pose.orientation.w = quat.w();

      keys.insert(value.key);
      graph_msg->poses.emplace_back(msg);
      continue;
    }

    const auto point_value = dynamic_cast<const gtsam::GenericValue<gtsam::Point3>*>(&value.value);
    if (point_value) {
      const gtsam::Point3& point = point_value->value();

      PointWithID msg;
      msg.type = PointWithID::NONE;
      msg.key = value.key;
      msg.point.x = point.x();
      msg.point.y = point.y();
      msg.point.z = point.z();

      keys.insert(value.key);
      graph_msg->points.emplace_back(msg);
    }
  }

  for (const auto& factor : graph) {
    switch (factor->keys().size()) {
      case 1:
        break;
      case 2:
        if (keys.find(factor->keys()[0]) != keys.end() && keys.find(factor->keys()[1]) != keys.end()) {
          BinaryFactor factor_msg;
          factor_msg.keys[0] = factor->keys()[0];
          factor_msg.keys[1] = factor->keys()[1];
          graph_msg->binary_factors.emplace_back(factor_msg);
        }
        break;
    }
  }

  return graph_msg;
}

}  // namespace factor_graph_interfaces