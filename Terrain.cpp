#include "Terrain.h"

namespace {
struct RGB
{
  char R{};
  char G{};
  char B{};
  bool operator==(const RGB& other) const { return R == other.R && G == other.G && B == other.B; }
};

/// Definitions of Terrain RGB Values as stored in the PPM Image Data
constexpr RGB TERRAIN_WATER = {static_cast<char>(158), static_cast<char>(217), static_cast<char>(246)};
constexpr RGB TERRAIN_LEVEL_1 = {static_cast<char>(203), static_cast<char>(226), static_cast<char>(163)};
constexpr RGB TERRAIN_LEVEL_2 = {static_cast<char>(255), static_cast<char>(250), static_cast<char>(188)};
constexpr RGB TERRAIN_LEVEL_3 = {static_cast<char>(251), static_cast<char>(203), static_cast<char>(114)};
constexpr RGB TERRAIN_LEVEL_4 = {static_cast<char>(222), static_cast<char>(163), static_cast<char>(83)};
constexpr RGB ROBOT_PATH = {static_cast<char>(255), static_cast<char>(0), static_cast<char>(0)};

inline TerrainValue from_rgb(const RGB& rgb)
{
  if (rgb == TERRAIN_WATER)
    return TerrainValue::WATER;
  if (rgb == TERRAIN_LEVEL_1)
    return TerrainValue::LEVEL_1;
  if (rgb == TERRAIN_LEVEL_2)
    return TerrainValue::LEVEL_2;
  if (rgb == TERRAIN_LEVEL_3)
    return TerrainValue::LEVEL_3;
  if (rgb == TERRAIN_LEVEL_4)
    return TerrainValue::LEVEL_4;
  if (rgb == ROBOT_PATH)
    return TerrainValue::ROBOT_PATH;

  return TerrainValue::UNKNOWN;
}

inline RGB to_rgb(const TerrainValue terrain_value)
{
  switch (terrain_value)
  {
  case TerrainValue::WATER:
    return TERRAIN_WATER;
  case TerrainValue::LEVEL_1:
    return TERRAIN_LEVEL_1;
  case TerrainValue::LEVEL_2:
    return TERRAIN_LEVEL_2;
  case TerrainValue::LEVEL_3:
    return TERRAIN_LEVEL_3;
  case TerrainValue::LEVEL_4:
    return TERRAIN_LEVEL_4;
  case TerrainValue::ROBOT_PATH:
    return ROBOT_PATH;
  case TerrainValue::UNKNOWN:
    return {0, 0, 0};
  }
  return {0, 0, 0};
}
} // namespace

Terrain::Terrain(const ppm::PPMObject& ppm_object) : width_(ppm_object.width), height_(ppm_object.height)
{
  terrain_.reserve(ppm_object.rgb_data.size() / 3U);
  for (std::size_t i = 0; i < ppm_object.rgb_data.size(); i += 3U)
  {
    terrain_.push_back(from_rgb({ppm_object.rgb_data[i], ppm_object.rgb_data[i + 1U], ppm_object.rgb_data[i + 2U]}));
  }
}

TerrainValue Terrain::get_value(const std::size_t x, const std::size_t y)
{
  const auto idx = x * width_ + y;
  if (idx > terrain_.size())
  {
    return TerrainValue::UNKNOWN;
  }
  return terrain_[x * width_ + y];
}

std::size_t Terrain::get_width() const
{
  return width_;
}
std::size_t Terrain::get_height() const
{
  return height_;
}

ppm::PPMObject Terrain::_get_ppm_with_path(const RobotPath& path)
{
  ppm::PPMObject obj;
  obj.width = width_;
  obj.height = height_;
  obj.maxColVal = 255;
  obj.rgb_data.reserve(width_ * height_ * 3U);
  for (const auto [x, y] : path)
  {
    terrain_[x * width_ + y] = TerrainValue::ROBOT_PATH;
  }
  for (const auto val : terrain_)
  {
    const auto rgb = to_rgb(val);
    obj.rgb_data.push_back(rgb.R);
    obj.rgb_data.push_back(rgb.G);
    obj.rgb_data.push_back(rgb.B);
  }
  return obj;
}
