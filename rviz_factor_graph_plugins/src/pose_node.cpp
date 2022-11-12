#include <pose_node.hpp>

#include <iostream>
#include <rviz_rendering/objects/axes.hpp>

#include <colormap.hpp>

namespace rviz_factor_graph_plugins {

struct ChannelOffsets {
  ChannelOffsets(const sensor_msgs::msg::PointCloud2& points_msg) : x_offset(-1), y_offset(-1), z_offset(-1), t_offset(-1), rgb_offset(-1), intensity_offset(-1) {
    std::vector<std::tuple<std::string, int*, std::uint8_t*>> typemap = {
      {"x", &x_offset, &x_datatype},
      {"y", &y_offset, &y_datatype},
      {"z", &z_offset, &z_datatype},
      {"t", &t_offset, &t_datatype},
      {"time", &t_offset, &t_datatype},
      {"rgb", &rgb_offset, &rgb_datatype},
      {"rgba", &rgb_offset, &rgb_datatype},
      {"intensity", &intensity_offset, &intensity_datatype},
    };

    for (const auto& field : points_msg.fields) {
      for (auto& [name, offset, datatype] : typemap) {
        if (field.name == name) {
          *offset = field.offset;
          *datatype = field.datatype;
        }
      }
    }
  }

  Ogre::Vector3 getPosition(const sensor_msgs::msg::PointCloud2& points_msg, int i) const {
    Ogre::Vector3 pos;
    pos.x = *reinterpret_cast<const float*>(points_msg.data.data() + points_msg.point_step * i + x_offset);
    pos.y = *reinterpret_cast<const float*>(points_msg.data.data() + points_msg.point_step * i + y_offset);
    pos.z = *reinterpret_cast<const float*>(points_msg.data.data() + points_msg.point_step * i + z_offset);
    return pos;
  }

  double getIntensity(const sensor_msgs::msg::PointCloud2& points_msg, int i) const {
    if (intensity_offset < 0) {
      return 0.0;
    }

    if (intensity_datatype == sensor_msgs::msg::PointField::FLOAT32) {
      return *reinterpret_cast<const float*>(points_msg.data.data() + points_msg.point_step * i + intensity_offset);
    } else if (intensity_datatype == sensor_msgs::msg::PointField::FLOAT64) {
      return *reinterpret_cast<const double*>(points_msg.data.data() + points_msg.point_step * i + intensity_offset);
    }

    return 0.0;
  }

  Ogre::ColourValue getColor(const sensor_msgs::msg::PointCloud2& points_msg, int i) const {
    if (rgb_offset < 0 || rgb_datatype != sensor_msgs::msg::PointField::UINT32) {
      return Ogre::ColourValue();
    }

    const std::uint8_t* rgb = points_msg.data.data() + points_msg.point_step * i + rgb_offset;
    return Ogre::ColourValue(rgb[0] / 255.0f, rgb[1] / 255.0f, rgb[2] / 255.0f);
  }

  int x_offset;
  int y_offset;
  int z_offset;
  int t_offset;
  int rgb_offset;
  int intensity_offset;

  std::uint8_t x_datatype;
  std::uint8_t y_datatype;
  std::uint8_t z_datatype;
  std::uint8_t t_datatype;
  std::uint8_t rgb_datatype;
  std::uint8_t intensity_datatype;
};

PoseNode::PoseNode(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node) {
  node = parent_node->createChildSceneNode();

  axes_node = node->createChildSceneNode();
  axes.reset(new rviz_rendering::Axes(scene_manager, axes_node, 0.5f, 0.05f));

  points_node = node->createChildSceneNode();
}
PoseNode::~PoseNode() {}

void PoseNode::setPointCloud(const sensor_msgs::msg::PointCloud2& points_msg, const std::shared_ptr<PointColorSettings>& color_settings) {
  const ChannelOffsets offsets(points_msg);
  const int num_points = points_msg.width * points_msg.height;

  std::vector<rviz_rendering::PointCloud::Point> points(num_points);
  for (int i = 0; i < num_points; i++) {
    points[i].position = offsets.getPosition(points_msg, i);
  }

  const Ogre::Vector3 position = getPosition();
  const Ogre::Quaternion orientation = getOrientation();

  std::vector<double> values(num_points, 0.0);
  if (color_settings->mode == PointColorSettings::AxisColor) {
    const int axis = static_cast<int>(color_settings->axis);
    for (int i = 0; i < num_points; i++) {
      const Ogre::Vector3 pos = orientation * points[i].position + position;
      values[i] = pos[axis];
    }
  } else if (color_settings->mode == PointColorSettings::Intensity) {
    for (int i = 0; i < num_points; i++) {
      values[i] = offsets.getIntensity(points_msg, i);
    }
  }

  if (color_settings->auto_range) {
    for (const double value : values) {
      color_settings->range_min = std::min(color_settings->range_min, value);
      color_settings->range_max = std::max(color_settings->range_max, value);
    }
  }

  const auto calcColor = [&](int i) {
    const double max = color_settings->range_max;
    const double min = color_settings->range_min;
    const double p = (values[i] - min) / (max - min);
    const int x = std::max<int>(0, std::min<int>(255, 255 * p));
    const auto& rgb = colormap(color_settings->colormap, x);
    return Ogre::ColourValue(rgb[0] / 255.0f, rgb[1] / 255.0f, rgb[2] / 255.0f, 1.0);
  };

  for (int i = 0; i < num_points; i++) {
    switch (color_settings->mode) {
      case PointColorSettings::AxisColor:
      case PointColorSettings::Intensity:
        points[i].color = calcColor(i);
        break;

      case PointColorSettings::FlatColor:
        points[i].color = color_settings->color;
        break;

      case PointColorSettings::RGB:
        points[i].color = offsets.getColor(points_msg, i);
        break;
    }
  }

  this->points = std::make_shared<rviz_rendering::PointCloud>();
  this->points->setName("points");
  this->points->addPoints(points.begin(), points.end());
  this->points->setRenderMode(rviz_rendering::PointCloud::RenderMode::RM_POINTS);
  this->color_settings = *color_settings;

  points_node->attachObject(this->points.get());
}

void PoseNode::setVisibility(bool show_axes, bool show_points) {
  axes_node->setVisible(show_axes);
  points_node->setVisible(show_points);
}

void PoseNode::setPose(const Ogre::Vector3& position, const Ogre::Quaternion& orientation) {
  if (points && recoloringRequired(position, orientation)) {
    points.reset();
  }

  node->setPosition(position);
  node->setOrientation(orientation);
}

Ogre::Vector3 PoseNode::getPosition() const {
  return node->getPosition();
}

Ogre::Quaternion PoseNode::getOrientation() const {
  return node->getOrientation();
}

void PoseNode::setAxesShape(float length, float radius) {  //
  axes->set(length, radius);
}

void PoseNode::setPointStyle(float size, float alpha, rviz_rendering::PointCloud::RenderMode mode) {
  if (points) {
    points->setRenderMode(mode);
    points->setAlpha(alpha);
    points->setDimensions(size, size, size);
  }
}

bool PoseNode::recoloringRequired(const Ogre::Vector3& position, const Ogre::Quaternion& orientation) {  //
  if (color_settings.mode == PointColorSettings::AxisColor) {
    if ((this->getPosition() - position).length() > 1e-2) {
      return true;
    }

    Ogre::Radian angle;
    Ogre::Vector3 axis;
    (this->getOrientation().Inverse() * orientation).ToAngleAxis(angle, axis);

    if (angle.valueDegrees() > 0.5f) {
      return true;
    }
  }

  return false;
}

bool PoseNode::recoloringRequired(const PointColorSettings& color_settings) {  //
  if (this->color_settings.mode != color_settings.mode) {
    return true;
  }

  switch (color_settings.mode) {
    case PointColorSettings::AxisColor:
      if (this->color_settings.axis != color_settings.axis) {
        return true;
      }
      if (std::abs(this->color_settings.range_min - color_settings.range_min) > 1e-3 || std::abs(this->color_settings.range_max - color_settings.range_max) > 1e-3) {
        return true;
      }
      return false;

    case PointColorSettings::FlatColor:
      return std::abs(this->color_settings.color.r - color_settings.color.r) > 1e-3 ||  //
             std::abs(this->color_settings.color.g - color_settings.color.g) > 1e-3 ||  //
             std::abs(this->color_settings.color.b - color_settings.color.b) > 1e-3;

    case PointColorSettings::Intensity:
      if (std::abs(this->color_settings.range_min - color_settings.range_min) > 1e-3 || std::abs(this->color_settings.range_max - color_settings.range_max) > 1e-3) {
        return true;
      }
    case PointColorSettings::RGB:
      return false;
  }

  return false;
}
}  // namespace rviz_factor_graph_plugins