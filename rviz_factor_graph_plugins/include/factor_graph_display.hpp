#pragma once

#include <rviz_common/message_filter_display.hpp>
#include <rviz_common/properties/float_property.hpp>

#include <factor_graph_interfaces/msg/factor_graph.hpp>
#include <factor_graph_visual.hpp>

namespace rviz_factor_graph_plugins {

class FactorGraphDisplay : public rviz_common::MessageFilterDisplay<factor_graph_interfaces::msg::FactorGraph> {
  Q_OBJECT

public:
  FactorGraphDisplay();
  virtual ~FactorGraphDisplay();

  virtual void onInitialize() override;
  virtual void reset() override;

  virtual void onEnable() override;
  virtual void onDisable() override;

  virtual void update(float wall_dt, float ros_dt) override;
  virtual void processMessage(factor_graph_interfaces::msg::FactorGraph::ConstSharedPtr graph_msg) override;

private Q_SLOTS:
  void updateShape();

private:
  std::shared_ptr<FactorGraphVisual> visual;

  rviz_common::properties::FloatProperty* length_property;
  rviz_common::properties::FloatProperty* radius_property;
};

}  // namespace rviz_factor_graph_plugins