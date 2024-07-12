#pragma once

#include "PPMData.h"

using RobotCoordinate = std::pair<std::size_t, std::size_t>;
using RobotPath = std::vector<RobotCoordinate>;

/// Terrain Description
enum class TerrainValue
{
  WATER, //< Robot can not move on Water
  LEVEL_1,
  LEVEL_2,
  LEVEL_3,
  LEVEL_4,
  ROBOT_PATH,
  UNKNOWN
};

/// Terrain Map describing the environment in which the robot is moving
class Terrain
{
public:
  /// Loads the terrain from a PPMObject
  explicit Terrain(const ppm::PPMObject& ppm_object);

  /// Returns the terrain value at the given coordinate
  /// @param x the x coordinate
  /// @param y the y coordinate
  /// @return the terrain value at the given coordinate
  TerrainValue get_value(const std::size_t x, const std::size_t y);

  /// Returns the width of the terrain
  /// @return the width of the terrain
  std::size_t get_width() const;

  /// Returns the height of the terrain
  /// @return the height of the terrain
  std::size_t get_height() const;

  /// Returns a PPMObject with the path drawn into it
  /// @param path the path to draw
  /// @return a PPMObject with the path drawn into it
  ppm::PPMObject _get_ppm_with_path(const RobotPath& path);

private:
  std::size_t width_{};
  std::size_t height_{};
  std::vector<TerrainValue> terrain_{};
};