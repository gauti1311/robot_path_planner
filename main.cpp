#include "PPMData.h"
#include "Planner.h"
#include "Terrain.h"

#include <chrono>
#include <cstdlib>
#include <iostream>

int main(int argc, char** argv)
{
  if (argc != 3)
  {
    std::cout << "Usage: ./robot_motion_planning <terrain_in> <terrain_out>" << std::endl;
    return EXIT_FAILURE;
  }
  const auto file_in = std::string(argv[1]);
  const auto file_out = std::string(argv[2]);

  // load the terrain image
  const auto ppm_object = ppm::load_file(file_in);
  if (!ppm_object.has_value())
  {
    std::cout << "Could not load terrain file" << std::endl;
    return EXIT_FAILURE;
  }

  // Terrain Map
  Terrain terrain{*ppm_object};
  std::cout << "Loaded Terrain with size: " << terrain.get_width() << " x " << terrain.get_height() << std::endl;

  // Robot Start and Destination
  RobotCoordinate start(30, 2026);
  RobotCoordinate destination = {2030, 25};

  Planner planner(&terrain);
  const auto plan_start = std::chrono::high_resolution_clock::now();

  /// ----- your planning code needs to be executed here ---///

  auto path = planner.plan_path_to_target(start, destination);

  /// ---------------- planning code end -------------------///

  const auto plan_time = std::chrono::high_resolution_clock::now() - plan_start;
  std::cout << "Planning Finished in " << std::chrono::duration_cast<std::chrono::milliseconds>(plan_time).count()
            << " ms" << std::endl;

  // writing the output image
  const auto ppm_object_out = terrain._get_ppm_with_path(path);
  ppm::write_file(file_out, ppm_object_out);

  return EXIT_SUCCESS;
}
