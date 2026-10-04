#pragma once

#include <filesystem>

#include "traj_cover/coverage_planner.hpp"

namespace traj_cover
{

class Visualizer
{
public:
  static void write_all(
    const PlanningResult & result,
    const std::filesystem::path & output_directory);
};

}  // namespace traj_cover
