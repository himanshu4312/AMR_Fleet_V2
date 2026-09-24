#ifndef AMR_PLANNER_PLUGINS__ASTAR_PLANNER_HPP_
#define AMR_PLANNER_PLUGINS__ASTAR_PLANNER_HPP_

#include <string>

#include "amr_planner_plugins/grid_search_planner.hpp"

namespace amr_planner_plugins
{

class AStarPlanner : public GridSearchPlanner
{
public:
  AStarPlanner() = default;
  ~AStarPlanner() override = default;

protected:
  double heuristic(const CellIndex & a, const CellIndex & b) const override;
  std::string plannerTypeName() const override {return "AStarPlanner";}
};

}

#endif
