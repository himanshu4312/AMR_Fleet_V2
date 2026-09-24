#include "amr_planner_plugins/astar_planner.hpp"

#include <cmath>
#include <algorithm>

namespace amr_planner_plugins
{

double AStarPlanner::heuristic(const CellIndex & a, const CellIndex & b) const
{

  double dx = std::abs(a.x - b.x);
  double dy = std::abs(a.y - b.y);
  double res = costmap_->getResolution();
  return res * ((dx + dy) + (std::sqrt(2.0) - 2.0) * std::min(dx, dy));
}

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(amr_planner_plugins::AStarPlanner, nav2_core::GlobalPlanner)
