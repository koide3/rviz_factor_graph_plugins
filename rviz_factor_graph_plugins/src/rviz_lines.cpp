#include <rviz_lines.hpp>

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreTechnique.h>

#include <rviz_rendering/material_manager.hpp>

namespace rviz_factor_graph_plugins {

Lines::Lines(Ogre::SceneManager* manager, Ogre::SceneNode* parent_node) : rviz_rendering::Object(manager) {
  if (!parent_node) {
    parent_node = manager->getRootSceneNode();
  }

  this->manual_object = manager->createManualObject();
  this->scene_node = parent_node->createChildSceneNode();

  static int count = 0;
  std::string line_material_name = "LinesMaterial" + std::to_string(count++);

  manual_object_material = rviz_rendering::MaterialManager::createMaterialWithLighting(line_material_name);
  manual_object_material->getTechnique(0)->getPass(0)->setDiffuse(0, 0, 0, 0);
  manual_object_material->getTechnique(0)->getPass(0)->setAmbient(1, 1, 1);

  this->scene_node->attachObject(manual_object);
}

Lines::~Lines() {
  if (scene_node->getParentSceneNode()) {
    scene_node->getParentSceneNode()->removeChild(scene_node);
  }

  scene_manager_->destroySceneNode(scene_node);
  scene_manager_->destroyManualObject(manual_object);
  manual_object_material->unload();
}

void Lines::setPoints(const std::vector<Ogre::Vector3>& points, bool line_strip) {
  manual_object->clear();
  manual_object->begin(manual_object_material->getName(), line_strip ? Ogre::RenderOperation::OT_LINE_STRIP : Ogre::RenderOperation::OT_LINE_LIST, "rviz_rendering");

  for (const auto& pt : points) {
    manual_object->position(pt);
  }

  manual_object->end();
  setVisible(true);
}

void Lines::setVisible(bool visible) {
  scene_node->setVisible(visible, true);
}

void Lines::setPosition(const Ogre::Vector3& position) {
  scene_node->setPosition(position);
}

void Lines::setOrientation(const Ogre::Quaternion& orientation) {
  scene_node->setOrientation(orientation);
}

void Lines::setScale(const Ogre::Vector3& scale) {
  scene_node->setScale(scale);
}

const Ogre::Vector3& Lines::getPosition() {
  return scene_node->getPosition();
}

const Ogre::Quaternion& Lines::getOrientation() {
  return scene_node->getOrientation();
}

void Lines::setUserData(const Ogre::Any& data) {
  manual_object->getUserObjectBindings().setUserAny(data);
}

void Lines::setColor(float r, float g, float b, float a) {
  setColor(Ogre::ColourValue(r, g, b, a));
}

void Lines::setColor(const Ogre::ColourValue& c) {
  manual_object_material->getTechnique(0)->setAmbient(c * 0.5);
  manual_object_material->getTechnique(0)->setDiffuse(c);

  rviz_rendering::MaterialManager::enableAlphaBlending(manual_object_material, c.a);
}

}  // namespace rviz_factor_graph_plugins