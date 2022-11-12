#pragma once

#include <array>
#include <vector>

namespace rviz_factor_graph_plugins {

enum class COLORMAP {
  TURBO = 0,
  JET,
  CIVIDIS,
  OCEAN,
  SPRING,
  SUMMER,
  AUTUMN,
  WINTER,
  GREAN_YELLOW,
  BLUE_RED,
  PUBUGN,
  TURBID,
  PASTEL,
  HELIX,
  PHASE,
  VEGETATION,
  CURL,
  COOL_WARM,
  NUM_COLORMAPS
};

std::vector<const char*> colormap_names();
std::array<std::array<unsigned char, 3>, 256> colormap_table(COLORMAP type);

const std::array<unsigned char, 3>& colormap(COLORMAP type, int x);

}