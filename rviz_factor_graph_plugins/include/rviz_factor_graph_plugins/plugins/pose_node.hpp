#pragma once

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreVector3.h>
#include <OgreQuaternion.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <rviz_rendering/objects/point_cloud.hpp>

#include <rviz_factor_graph_plugins/plugins/point_color_settings.hpp>

namespace rviz_rendering {
class Axes;
}  // namespace rviz_rendering

namespace rviz_factor_graph_plugins {

struct PoseNode {
  PoseNode(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node);
  ~PoseNode();

  Ogre::Vector3 getPosition() const;
  Ogre::Quaternion getOrientation() const;

  void setVisibility(bool show_axes, bool show_points);

  void setPose(const Ogre::Vector3& position, const Ogre::Quaternion& orientation);
  void setPointCloud(const sensor_msgs::msg::PointCloud2& points_msg, const std::shared_ptr<PointColorSettings>& color_settings);
  void setAxesShape(float length, float radius);
  void setPointStyle(float size, float alpha, rviz_rendering::PointCloud::RenderMode mode);

  bool recoloringRequired(const Ogre::Vector3& position, const Ogre::Quaternion& orientation);
  bool recoloringRequired(const PointColorSettings& color_settings);

  Ogre::SceneNode* node;

  Ogre::SceneNode* axes_node;
  std::shared_ptr<rviz_rendering::Axes> axes;

  Ogre::SceneNode* points_node;
  std::shared_ptr<rviz_rendering::PointCloud> points;
  std::shared_ptr<rviz_rendering::PointCloud> points_bg;

  PointColorSettings color_settings;
};

}