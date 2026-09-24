#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "amr_planner_plugins/astar_planner.hpp"
#include "amr_planner_plugins/dijkstra_planner.hpp"
#include "amr_planner_plugins/rrt_planner.hpp"
#include "planner_test_utils.hpp"

namespace
{

using amr_planner_plugins_test::CostmapOptions;
using amr_planner_plugins_test::makePose;
using amr_planner_plugins_test::makeTestCostmap;

enum class PlannerKind { kAStar, kDijkstra, kRRT };

std::string plannerKindName(PlannerKind kind)
{
  switch (kind) {
    case PlannerKind::kAStar: return "A*";
    case PlannerKind::kDijkstra: return "Dijkstra";
    case PlannerKind::kRRT: return "RRT";
  }
  return "?";
}

nav2_core::GlobalPlanner::Ptr makePlanner(PlannerKind kind)
{
  switch (kind) {
    case PlannerKind::kAStar: return std::make_shared<amr_planner_plugins::AStarPlanner>();
    case PlannerKind::kDijkstra: return std::make_shared<amr_planner_plugins::DijkstraPlanner>();
    case PlannerKind::kRRT: return std::make_shared<amr_planner_plugins::RRTPlanner>();
  }
  return nullptr;
}

struct Scenario
{
  std::string name;
  CostmapOptions costmap_opts;
  geometry_msgs::msg::PoseStamped start;
  geometry_msgs::msg::PoseStamped goal;
  std::function<void(nav2_costmap_2d::Costmap2D *)> build_obstacles;
};

struct TrialResult
{
  bool success{false};
  double planning_time_ms{0.0};
  double path_length{0.0};
};

std::atomic<int> g_node_counter{0};

TrialResult runTrial(PlannerKind kind, const Scenario & scenario, int trial_index)
{
  auto costmap_ros = makeTestCostmap(
    "bench_costmap_" + std::to_string(g_node_counter++), scenario.costmap_opts);
  auto * costmap = costmap_ros->getCostmap();
  if (scenario.build_obstacles) {
    scenario.build_obstacles(costmap);
  }

  if (kind == PlannerKind::kRRT) {
    costmap_ros->declare_parameter(
      "bench.rng_seed", rclcpp::ParameterValue(1000 + trial_index));
  }

  auto planner = makePlanner(kind);
  planner->configure(
    std::weak_ptr<rclcpp_lifecycle::LifecycleNode>(costmap_ros), "bench",
    costmap_ros->getTfBuffer(), costmap_ros);
  planner->activate();

  TrialResult result;
  auto t0 = std::chrono::steady_clock::now();
  try {
    auto path = planner->createPlan(scenario.start, scenario.goal);
    result.success = true;
    result.path_length = amr_planner_plugins_test::pathLength(path);
  } catch (const std::exception &) {
    result.success = false;
  }
  auto t1 = std::chrono::steady_clock::now();
  result.planning_time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

  planner->deactivate();
  planner->cleanup();
  costmap_ros->on_cleanup(rclcpp_lifecycle::State());
  return result;
}

std::vector<Scenario> buildScenarios()
{
  std::vector<Scenario> scenarios;

  {
    Scenario s;
    s.name = "Open space (20x20m, no obstacles)";
    s.costmap_opts = CostmapOptions{20, 20, 0.1, 0.0, 0.0, false};
    s.start = makePose(1.0, 1.0);
    s.goal = makePose(19.0, 19.0);
    scenarios.push_back(s);
  }

  {
    Scenario s;
    s.name = "Narrow corridor (0.5m gap, 20x10m)";
    s.costmap_opts = CostmapOptions{20, 10, 0.1, 0.0, 0.0, false};
    s.start = makePose(1.0, 5.0);
    s.goal = makePose(19.0, 5.0);
    s.build_obstacles = [](nav2_costmap_2d::Costmap2D * costmap) {

        unsigned int x0_mx, x1_mx, dummy_my;
        if (!costmap->worldToMap(3.0, 0.0, x0_mx, dummy_my)) {return;}
        if (!costmap->worldToMap(17.0, 0.0, x1_mx, dummy_my)) {return;}
        for (unsigned int mx = x0_mx; mx <= x1_mx; ++mx) {
          for (unsigned int my = 0; my < costmap->getSizeInCellsY(); ++my) {
            double wx, wy;
            costmap->mapToWorld(mx, my, wx, wy);
            if (wy < 4.75 || wy > 5.25) {
              costmap->setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
            }
          }
        }
      };
    scenarios.push_back(s);
  }

  {
    Scenario s;
    s.name = "Maze-like obstacle field (20x20m, 0.6m gaps)";
    s.costmap_opts = CostmapOptions{20, 20, 0.1, 0.0, 0.0, false};
    s.start = makePose(1.0, 1.0);
    s.goal = makePose(19.0, 19.0);
    s.build_obstacles = [](nav2_costmap_2d::Costmap2D * costmap) {

        const double xs[] = {4.0, 8.0, 12.0, 16.0};
        const double kGap = 0.6;
        bool gap_at_top = true;
        for (double x : xs) {
          double gap_before = gap_at_top ? 0.0 : kGap;
          double gap_after = gap_at_top ? kGap : 0.0;
          amr_planner_plugins_test::addWallWithGaps(costmap, x, gap_before, gap_after);
          gap_at_top = !gap_at_top;
        }
      };
    scenarios.push_back(s);
  }

  {
    Scenario s;
    s.name = "Small costmap (5x5m), short hop";
    s.costmap_opts = CostmapOptions{5, 5, 0.1, 0.0, 0.0, false};
    s.start = makePose(1.0, 1.0);
    s.goal = makePose(4.0, 4.0);
    scenarios.push_back(s);
  }

  {
    Scenario s;
    s.name = "Large costmap (50x50m), same short hop";
    s.costmap_opts = CostmapOptions{50, 50, 0.1, 0.0, 0.0, false};
    s.start = makePose(1.0, 1.0);
    s.goal = makePose(4.0, 4.0);
    scenarios.push_back(s);
  }

  return scenarios;
}

struct Aggregate
{
  int successes{0};
  int trials{0};
  double total_time_ms{0.0};
  double total_length{0.0};
};

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  const std::vector<PlannerKind> kinds = {
    PlannerKind::kAStar, PlannerKind::kDijkstra, PlannerKind::kRRT};
  auto scenarios = buildScenarios();

  std::cout << std::left
            << std::setw(42) << "Scenario"
            << std::setw(10) << "Planner"
            << std::setw(12) << "Success"
            << std::setw(16) << "Avg time (ms)"
            << std::setw(14) << "Avg length (m)"
            << "\n";
  std::cout << std::string(94, '-') << "\n";

  for (const auto & scenario : scenarios) {
    for (auto kind : kinds) {

      int trial_count = (kind == PlannerKind::kRRT) ? 10 : 1;

      Aggregate agg;
      for (int t = 0; t < trial_count; ++t) {
        TrialResult r = runTrial(kind, scenario, t);
        agg.trials++;
        agg.total_time_ms += r.planning_time_ms;
        if (r.success) {
          agg.successes++;
          agg.total_length += r.path_length;
        }
      }

      double success_rate = 100.0 * agg.successes / agg.trials;
      double avg_time = agg.total_time_ms / agg.trials;
      double avg_length = agg.successes > 0 ? agg.total_length / agg.successes : 0.0;

      std::cout << std::left << std::fixed << std::setprecision(3)
                << std::setw(42) << scenario.name
                << std::setw(10) << plannerKindName(kind)
                << std::setw(12) << (std::to_string(agg.successes) + "/" +
      std::to_string(agg.trials) + " (" + std::to_string(static_cast<int>(success_rate)) + "%)")
                << std::setw(16) << avg_time
                << std::setw(14) << (agg.successes > 0 ? std::to_string(avg_length) : "-")
                << "\n";
    }
    std::cout << std::string(94, '-') << "\n";
  }

  rclcpp::shutdown();
  return 0;
}
