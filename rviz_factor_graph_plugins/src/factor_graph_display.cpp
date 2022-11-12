#include <factor_graph_display.hpp>

#include <colormap.hpp>

namespace rviz_factor_graph_plugins {

using GetPointCloud = factor_graph_interfaces::srv::GetPointCloud;

FactorGraphDisplay::FactorGraphDisplay() {
  using namespace rviz_common::properties;

  color_settings.reset(new PointColorSettings);

  // Factor-related properties
  show_factors_property = new BoolProperty("ShowFactors", true, "Visibility of factors.", this, SLOT(updateVisibility()));
  factor_color_property = new ColorProperty("FactorColor", QColor(0, 255, 0, 255), "Color to paint factors.", show_factors_property, SLOT(updateVisibility()), this);
  factor_alpha_property = new FloatProperty("FactorAlpha", 0.2f, "Transparency of factors.", show_factors_property, SLOT(updateVisibility()), this);

  // Axes-related properties
  show_axes_property = new BoolProperty("ShowAxes", true, "Visibility of axes.", this, SLOT(updateVisibility()));
  length_property = new FloatProperty("AxesLength", 1.0f, "Length of each axis, in meters.", show_axes_property, SLOT(updateVisibility()), this);
  radius_property = new FloatProperty("AxesRadius", 0.1f, "Radius of each axis, in meters.", show_axes_property, SLOT(updateVisibility()), this);

  // Points-related properties
  show_points_property = new BoolProperty("ShowPoints", true, "Visibility of points.", this, SLOT(updateVisibility()));
  point_size_property = new FloatProperty("PointSize", 0.05f, "Radius of each point, in meters.", show_points_property, SLOT(updatePointStyle()), this);
  point_alpha_property = new FloatProperty("PointAlpha", 0.8f, "Transparency of each point.", show_points_property, SLOT(updatePointStyle()), this);
  point_style_property = new EnumProperty("PointStyle", "Flat Squares", "Rendering mode to use.", show_points_property, SLOT(updatePointStyle()), this);
  point_style_property->addOption("Points", rviz_rendering::PointCloud::RM_POINTS);
  point_style_property->addOption("Squares", rviz_rendering::PointCloud::RM_SQUARES);
  point_style_property->addOption("Flat Squares", rviz_rendering::PointCloud::RM_FLAT_SQUARES);
  point_style_property->addOption("Spheres", rviz_rendering::PointCloud::RM_SPHERES);
  point_style_property->addOption("Boxes", rviz_rendering::PointCloud::RM_BOXES);
  point_style_property->addOption("Tiles", rviz_rendering::PointCloud::RM_TILES);

  point_color_property = new EnumProperty("PointColorTransformer", "AxisColor", "Point color transformer", show_points_property, SLOT(resetRange()), this);
  point_color_property->addOption("AxisColor", PointColorSettings::AxisColor);
  point_color_property->addOption("FlatColor", PointColorSettings::FlatColor);
  point_color_property->addOption("Intensity", PointColorSettings::Intensity);
  point_color_property->addOption("RGB", PointColorSettings::RGB);

  axis_property = new EnumProperty("AxisColor", "z", "Point coloring axis.", show_points_property, SLOT(resetRange()), this);
  axis_property->addOption("x", PointColorSettings::X);
  axis_property->addOption("y", PointColorSettings::Y);
  axis_property->addOption("z", PointColorSettings::Z);

  colormap_property = new EnumProperty("ColorMap", "TURBO", "Colormap for AxisColor.", show_points_property, SLOT(updatePointColor()), this);
  const auto colormaps = colormap_names();
  for (int i = 0; i < colormaps.size(); i++) {
    colormap_property->addOption(colormaps[i], i);
  }

  auto_range_property = new BoolProperty("AutoRange", true, "Auto compute axis range.", show_points_property, SLOT(resetRange()), this);
  range_min_property = new FloatProperty("RangeMin", -10.0f, "AxisColor range.", show_points_property, SLOT(updatePointColor()), this);
  range_max_property = new FloatProperty("RangeMax", 10.0f, "AxisColor range.", show_points_property, SLOT(updatePointColor()), this);
  flat_color_property = new ColorProperty("FlatColor", QColor(255, 255, 255, 255), "Color to paint points.", show_points_property, SLOT(updatePointColor()), this);

  frame_id_property = new StringProperty("FrameID", "", "FrameID (readonly)", this);
  num_poses_property = new IntProperty("Poses", 0, "Number of pose variables in the graph message.", frame_id_property);
  num_points_property = new IntProperty("Points", 0, "Number of point variables in the graph message.", frame_id_property);
  num_unary_factors = new IntProperty("UnaryFactors", 0, "Number of unary factors in the graph message.", frame_id_property);
  num_binary_factors = new IntProperty("BinaryFactors", 0, "Number of binary factors in the graph message.", frame_id_property);

  service_property = new StringProperty("Service", "/get_point_cloud", "GetPointCloud service", this, SLOT(upateGetPointCloudService()));

  max_requests_property = new IntProperty("MaxRequests", 5, "Maximum number of GetPointCloud calls to be queued.", service_property, SLOT(updatePointsLoadingParams()), this);
  max_load_count_property = new IntProperty("MaxLoadCount", 15, "Maximum count for pose node update check.", service_property, SLOT(updatePointsLoadingParams()), this);
}

FactorGraphDisplay::~FactorGraphDisplay() {}

void FactorGraphDisplay::onInitialize() {
  MFDClass::onInitialize();
  visual.reset(new FactorGraphVisual(context_->getSceneManager(), scene_node_));

  updateVisibility();
  updatePointStyle();
  updatePointColor();
  upateGetPointCloudService();
}

void FactorGraphDisplay::reset() {
  MFDClass::reset();
}

void FactorGraphDisplay::update(float wall_dt, float ros_dt) {
  visual->update();

  if (color_settings->auto_range) {
    range_min_property->setFloat(color_settings->range_min);
    range_max_property->setFloat(color_settings->range_max);
  }
}

void FactorGraphDisplay::processMessage(factor_graph_interfaces::msg::FactorGraph::ConstSharedPtr graph_msg) {
  Ogre::Vector3 position;
  Ogre::Quaternion orientation;

  if (!context_->getFrameManager()->getTransform(graph_msg->header, position, orientation)) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("rviz_factor_graph_plugins"), "Failed to find transform between " << graph_msg->header.frame_id << " and " << fixed_frame_.toStdString());
    return;
  }

  visual->setPose(position, orientation);
  visual->setMessage(graph_msg);

  frame_id_property->setStdString(graph_msg->header.frame_id);
  num_poses_property->setInt(graph_msg->poses.size());
  num_points_property->setInt(graph_msg->points.size());
  num_unary_factors->setInt(graph_msg->unary_factors.size());
  num_binary_factors->setInt(graph_msg->binary_factors.size());
}

void FactorGraphDisplay::updateVisibility() {
  const bool show_factors = show_factors_property->getBool();
  const bool show_axes = show_axes_property->getBool();
  const bool show_points = show_points_property->getBool();
  visual->setVisibility(show_factors, show_axes, show_points);

  auto factor_color = factor_color_property->getOgreColor();
  factor_color[3] = factor_alpha_property->getFloat();
  visual->setFactorColor(factor_color);

  visual->setAxesShape(length_property->getFloat(), radius_property->getFloat());
}

void FactorGraphDisplay::updatePointStyle() {
  float size = point_size_property->getFloat();
  float alpha = point_alpha_property->getFloat();
  auto mode = static_cast<rviz_rendering::PointCloud::RenderMode>(point_style_property->getOptionInt());
  visual->setPointStyle(size, alpha, mode);
}

void FactorGraphDisplay::resetRange() {
  if (auto_range_property->getBool()) {
    color_settings->range_min = 0.0;
    color_settings->range_max = 0.0;
    range_min_property->setFloat(color_settings->range_min);
    range_max_property->setFloat(color_settings->range_max);
  }

  updatePointColor();
}

void FactorGraphDisplay::updatePointColor() {
  color_settings->mode = static_cast<PointColorSettings::ColorMode>(point_color_property->getOptionInt());
  color_settings->axis = static_cast<PointColorSettings::ColorAxis>(axis_property->getOptionInt());
  color_settings->colormap = static_cast<COLORMAP>(colormap_property->getOptionInt());
  color_settings->color = flat_color_property->getOgreColor();

  color_settings->auto_range = auto_range_property->getBool();
  if (!color_settings->auto_range) {
    color_settings->range_min = range_min_property->getFloat();
    color_settings->range_max = range_max_property->getFloat();
  }

  visual->setColorSettings(color_settings);
}

void FactorGraphDisplay::updatePointsLoadingParams() {
  int max_requests = max_requests_property->getInt();
  int max_load_count = max_load_count_property->getInt();
  visual->setPointsLoadingParams(max_load_count, max_requests);
}

void FactorGraphDisplay::upateGetPointCloudService() {
  auto node = rviz_ros_node_.lock();

  const std::string service_name = service_property->getString().toStdString();
  visual->setGetPointCloudService(node->get_raw_node(), node->get_raw_node()->create_client<GetPointCloud>(service_name));
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