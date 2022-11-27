#pragma once

#include <rviz_common/message_filter_display.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/color_property.hpp>

#include <factor_graph_interfaces/msg/factor_graph.hpp>

#include <rviz_factor_graph_plugins/plugins/factor_graph_visual.hpp>
#include <rviz_factor_graph_plugins/plugins/point_color_settings.hpp>

namespace rviz_factor_graph_plugins {

class FactorGraphDisplay : public rviz_common::MessageFilterDisplay<factor_graph_interfaces::msg::FactorGraph> {
  Q_OBJECT

public:
  using FactorGraph = factor_graph_interfaces::msg::FactorGraph;

  FactorGraphDisplay();
  virtual ~FactorGraphDisplay();

  virtual void onInitialize() override;
  virtual void reset() override;

  virtual void onEnable() override;
  virtual void onDisable() override;

  virtual void update(float wall_dt, float ros_dt) override;
  virtual void processMessage(FactorGraph::ConstSharedPtr graph_msg) override;

private Q_SLOTS:
  void updateVisibility();
  void updatePointStyle();
  void resetRange();
  void updatePointColor();
  void updatePointsLoadingParams();
  void upateGetPointCloudService();

private:
  std::shared_ptr<FactorGraphVisual> visual;
  std::shared_ptr<PointColorSettings> color_settings;

  rviz_common::properties::BoolProperty* show_factors_property;
  rviz_common::properties::ColorProperty* factor_color_property;
  rviz_common::properties::FloatProperty* factor_alpha_property;

  rviz_common::properties::BoolProperty* show_axes_property;
  rviz_common::properties::FloatProperty* length_property;
  rviz_common::properties::FloatProperty* radius_property;

  rviz_common::properties::BoolProperty* show_points_property;
  rviz_common::properties::FloatProperty* point_size_property;
  rviz_common::properties::FloatProperty* point_alpha_property;
  rviz_common::properties::EnumProperty* point_color_property;
  rviz_common::properties::EnumProperty* point_style_property;

  rviz_common::properties::EnumProperty* colormap_property;
  rviz_common::properties::EnumProperty* axis_property;
  rviz_common::properties::BoolProperty* auto_range_property;
  rviz_common::properties::FloatProperty* range_min_property;
  rviz_common::properties::FloatProperty* range_max_property;
  rviz_common::properties::ColorProperty* flat_color_property;

  rviz_common::properties::StringProperty* frame_id_property;
  rviz_common::properties::IntProperty* num_poses_property;
  rviz_common::properties::IntProperty* num_points_property;
  rviz_common::properties::IntProperty* num_unary_factors;
  rviz_common::properties::IntProperty* num_binary_factors;

  rviz_common::properties::StringProperty* service_property;
  rviz_common::properties::IntProperty* max_requests_property;
  rviz_common::properties::IntProperty* max_load_count_property;
};

}  // namespace rviz_factor_graph_plugins