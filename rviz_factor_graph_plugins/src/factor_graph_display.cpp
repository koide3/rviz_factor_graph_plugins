#include <factor_graph_display.hpp>

namespace rviz_factor_graph_plugins {

FactorGraphDisplay::FactorGraphDisplay() {
  length_property = new rviz_common::properties::FloatProperty("Axes length", 0.5f, "Length of each axis, in meters.", this, SLOT(updateShape()));
  radius_property = new rviz_common::properties::FloatProperty("Axes radius", 0.02f, "Radius of each axis, in meters.", this, SLOT(updateShape()));
}

FactorGraphDisplay::~FactorGraphDisplay() {}

void FactorGraphDisplay::onInitialize() {
  MFDClass::onInitialize();
  visual.reset(new FactorGraphVisual(context_->getSceneManager(), scene_node_));

  visual->setGetPointCloudService(rviz_ros_node_.lock()->get_raw_node()->create_client<factor_graph_interfaces::srv::GetPointCloud>("/get_point_cloud"));
}

void FactorGraphDisplay::reset() {
  MFDClass::reset();
}

void FactorGraphDisplay::update(float wall_dt, float ros_dt) {
  visual->update();
}

void FactorGraphDisplay::processMessage(factor_graph_interfaces::msg::FactorGraph::ConstSharedPtr graph_msg) {
  visual->setMessage(graph_msg);
}

void FactorGraphDisplay::updateShape() {
  visual->setAxesShape(length_property->getFloat(), radius_property->getFloat());
}

void FactorGraphDisplay::onEnable() {
  subscribe();
}

void FactorGraphDisplay::onDisable() {
  unsubscribe();
  visual->reset();
}

}  // namespace rviz_factor_graph_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(rviz_factor_graph_plugins::FactorGraphDisplay, rviz_common::Display)