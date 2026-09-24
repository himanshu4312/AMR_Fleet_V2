#ifndef AMR_PLANNER_PLUGINS__DIJKSTRA_PLANNER_HPP_
#define AMR_PLANNER_PLUGINS__DIJKSTRA_PLANNER_HPP_

#include <string>

#include "amr_planner_plugins/grid_search_planner.hpp"

namespace amr_planner_plugins
{

class DijkstraPlanner : public GridSearchPlanner
{
public:
  DijkstraPlanner() = default;
  ~DijkstraPlanner() override = default;

protected:
  double heuristic(const CellIndex &, const CellIndex &) const override {return 0.0;}
  std::string plannerTypeName() const override {return "DijkstraPlanner";}
};

}

#endif
