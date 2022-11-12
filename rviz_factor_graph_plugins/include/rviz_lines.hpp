#pragma once

#include <OgreSceneNode.h>
#include <OgreMaterial.h>
#include <OgreSharedPtr.h>

#include <rviz_rendering/objects/object.hpp>
#include <rviz_rendering/visibility_control.hpp>

namespace Ogre {
class SceneManager;
class SceneNode;
class Quaternion;
class Any;
class ColourValue;
}  // namespace Ogre

namespace rviz_factor_graph_plugins {

class Lines : public rviz_rendering::Object {
public:
  RVIZ_RENDERING_PUBLIC
  explicit Lines(Ogre::SceneManager* manager, Ogre::SceneNode* parent_node = nullptr);

  RVIZ_RENDERING_PUBLIC
  virtual ~Lines();

  RVIZ_RENDERING_PUBLIC
  void clear();

  RVIZ_RENDERING_PUBLIC
  void setPoints(const std::vector<Ogre::Vector3>& points, bool line_strip);

  RVIZ_RENDERING_PUBLIC
  void setVisible(bool visible);

  RVIZ_RENDERING_PUBLIC
  void setPosition(const Ogre::Vector3& position) override;

  RVIZ_RENDERING_PUBLIC
  void setOrientation(const Ogre::Quaternion& orientation) override;

  RVIZ_RENDERING_PUBLIC
  void setScale(const Ogre::Vector3& scale) override;

  RVIZ_RENDERING_PUBLIC
  const Ogre::Vector3& getPosition() override;

  RVIZ_RENDERING_PUBLIC
  const Ogre::Quaternion& getOrientation() override;

  RVIZ_RENDERING_PUBLIC
  void setUserData(const Ogre::Any& data) override;

  RVIZ_RENDERING_PUBLIC
  void setColor(float r, float g, float b, float a) override;

  RVIZ_RENDERING_PUBLIC
  virtual void setColor(const Ogre::ColourValue& c);

protected:
  Ogre::SceneNode* scene_node;
  Ogre::ManualObject* manual_object;
  Ogre::MaterialPtr manual_object_material;
};

}  // namespace rviz_factor_graph_plugins