#ifndef AMR_PLANNER_PLUGINS__COSTMAP_COST_UTILS_HPP_
#define AMR_PLANNER_PLUGINS__COSTMAP_COST_UTILS_HPP_

#include "nav2_costmap_2d/cost_values.hpp"

namespace amr_planner_plugins
{

inline bool isLethalCost(unsigned char cost, bool allow_unknown)
{
  if (cost == nav2_costmap_2d::LETHAL_OBSTACLE ||
    cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
  {
    return true;
  }
  return cost == nav2_costmap_2d::NO_INFORMATION && !allow_unknown;
}

}

#endif
