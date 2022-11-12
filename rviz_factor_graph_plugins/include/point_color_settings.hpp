#pragma once

#include <string>
#include <OgreVector3.h>
#include <OgreQuaternion.h>
#include <OgreColourValue.h>
#include <colormap.hpp>

namespace rviz_factor_graph_plugins {

struct PointColorSettings {
  enum ColorMode { AxisColor, FlatColor, Intensity, RGB };
  enum ColorAxis { X, Y, Z };

  PointColorSettings() {
    mode = FlatColor;
    axis = X;
    colormap = COLORMAP::TURBO;
    color = Ogre::ColourValue(1.0f, 1.0f, 1.0f, 1.0f);

    auto_range = true;
    range_min = 0.0;
    range_max = 0.0;
  }

  ColorMode mode;
  ColorAxis axis;
  COLORMAP colormap;
  Ogre::ColourValue color;

  bool auto_range;
  double range_min;
  double range_max;
};

}  // namespace rviz_factor_graph_plugins