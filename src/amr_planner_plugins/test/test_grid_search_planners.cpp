#include <memory>
#include <atomic>

#include "gtest/gtest.h"
#include "nav2_core/exceptions.hpp"
#include "amr_planner_plugins/astar_planner.hpp"
#include "amr_planner_plugins/dijkstra_planner.hpp"
#include "planner_test_utils.hpp"

namespace
{
using amr_planner_plugins_test::CostmapOptions;
using amr_planner_plugins_test::makeTestCostmap;
using amr_planner_plugins_test::makePose;

std::atomic<int> g_node_counter{0};

template<typename PlannerT>
class GridSearchPlannerTest : public ::testing::Test
{
protected:
  static constexpr const char * kPlannerName = "test_planner";

  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  PlannerT planner_;

  void configurePlanner(const CostmapOptions & opts, bool allow_unknown = true)
  {
    costmap_ros_ = makeTestCostmap(
      "grid_test_costmap_" + std::to_string(g_node_counter++), opts);
    costmap_ros_->declare_parameter(
      std::string(kPlannerName) + ".allow_unknown", rclcpp::ParameterValue(allow_unknown));
    planner_.configure(
      std::weak_ptr<rclcpp_lifecycle::LifecycleNode>(costmap_ros_), kPlannerName,
      costmap_ros_->getTfBuffer(), costmap_ros_);
    planner_.activate();
  }

  void TearDown() override
  {
    if (costmap_ros_) {
      planner_.deactivate();
      planner_.cleanup();
      costmap_ros_->on_cleanup(rclcpp_lifecycle::State());
    }
  }
};

using GridSearchPlannerTypes =
  ::testing::Types<amr_planner_plugins::AStarPlanner, amr_planner_plugins::DijkstraPlanner>;
TYPED_TEST_SUITE(GridSearchPlannerTest, GridSearchPlannerTypes);

TYPED_TEST(GridSearchPlannerTest, ValidPathOnOpenCostmapIsCollisionFreeAndConnected)
{
  this->configurePlanner(CostmapOptions{10, 10, 0.1, 0.0, 0.0, false});
  auto start = makePose(1.0, 5.0);
  auto goal = makePose(9.0, 5.0);

  nav_msgs::msg::Path path;
  ASSERT_NO_THROW(path = this->planner_.createPlan(start, goal));
  ASSERT_FALSE(path.poses.empty());

  auto * costmap = this->costmap_ros_->getCostmap();
  EXPECT_TRUE(amr_planner_plugins_test::pathIsCollisionFree(costmap, path, true));

  const auto & first = path.poses.front().pose.position;
  const auto & last = path.poses.back().pose.position;
  EXPECT_NEAR(first.x, start.pose.position.x, 0.1);
  EXPECT_NEAR(first.y, start.pose.position.y, 0.1);
  EXPECT_NEAR(last.x, goal.pose.position.x, 0.1);
  EXPECT_NEAR(last.y, goal.pose.position.y, 0.1);
}

TYPED_TEST(GridSearchPlannerTest, ValidPathAvoidsWallObstacle)
{
  this->configurePlanner(CostmapOptions{10, 10, 0.1, 0.0, 0.0, false});
  auto * costmap = this->costmap_ros_->getCostmap();
  amr_planner_plugins_test::addWallWithGaps(costmap, 5.0, 2.0, 2.0);

  auto start = makePose(1.0, 5.0);
  auto goal = makePose(9.0, 5.0);

  nav_msgs::msg::Path path;
  ASSERT_NO_THROW(path = this->planner_.createPlan(start, goal));
  EXPECT_TRUE(amr_planner_plugins_test::pathIsCollisionFree(costmap, path, true));
}

TYPED_TEST(GridSearchPlannerTest, ThrowsWhenGoalUnreachable)
{
  this->configurePlanner(CostmapOptions{10, 10, 0.1, 0.0, 0.0, false});
  auto * costmap = this->costmap_ros_->getCostmap();

  amr_planner_plugins_test::sealBoxAround(costmap, 5.0, 5.0, 0.3);

  auto start = makePose(1.0, 1.0);
  auto goal = makePose(5.0, 5.0);
  EXPECT_THROW(this->planner_.createPlan(start, goal), nav2_core::PlannerException);
}

TYPED_TEST(GridSearchPlannerTest, ThrowsWhenGoalCellIsLethal)
{
  this->configurePlanner(CostmapOptions{10, 10, 0.1, 0.0, 0.0, false});
  auto * costmap = this->costmap_ros_->getCostmap();
  unsigned int mx, my;
  ASSERT_TRUE(costmap->worldToMap(5.0, 5.0, mx, my));
  costmap->setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);

  auto start = makePose(1.0, 1.0);
  auto goal = makePose(5.0, 5.0);
  EXPECT_THROW(this->planner_.createPlan(start, goal), nav2_core::PlannerException);
}

TYPED_TEST(GridSearchPlannerTest, ThrowsWhenStartOutsideCostmapBounds)
{
  this->configurePlanner(CostmapOptions{10, 10, 0.1, 0.0, 0.0, false});
  auto start = makePose(-1000.0, -1000.0);
  auto goal = makePose(5.0, 5.0);
  EXPECT_THROW(this->planner_.createPlan(start, goal), nav2_core::PlannerException);
}

TYPED_TEST(GridSearchPlannerTest, ThrowsWhenGoalOutsideCostmapBounds)
{
  this->configurePlanner(CostmapOptions{10, 10, 0.1, 0.0, 0.0, false});
  auto start = makePose(5.0, 5.0);
  auto goal = makePose(1000.0, 1000.0);
  EXPECT_THROW(this->planner_.createPlan(start, goal), nav2_core::PlannerException);
}

TYPED_TEST(GridSearchPlannerTest, HandlesStartEqualsGoal)
{
  this->configurePlanner(CostmapOptions{10, 10, 0.1, 0.0, 0.0, false});
  auto pose = makePose(5.0, 5.0);

  nav_msgs::msg::Path path;
  ASSERT_NO_THROW(path = this->planner_.createPlan(pose, pose));
  ASSERT_EQ(path.poses.size(), 1u);
  EXPECT_NEAR(path.poses.front().pose.position.x, pose.pose.position.x, 0.1);
  EXPECT_NEAR(path.poses.front().pose.position.y, pose.pose.position.y, 0.1);
}

TYPED_TEST(GridSearchPlannerTest, AllUnknownCostmapSucceedsWhenAllowUnknownTrue)
{
  this->configurePlanner(CostmapOptions{10, 10, 0.1, 0.0, 0.0, true},   true);
  auto start = makePose(1.0, 1.0);
  auto goal = makePose(9.0, 9.0);
  EXPECT_NO_THROW(this->planner_.createPlan(start, goal));
}

TYPED_TEST(GridSearchPlannerTest, AllUnknownCostmapThrowsWhenAllowUnknownFalse)
{
  this->configurePlanner(CostmapOptions{10, 10, 0.1, 0.0, 0.0, true},   false);
  auto start = makePose(1.0, 1.0);
  auto goal = makePose(9.0, 9.0);

  EXPECT_THROW(this->planner_.createPlan(start, goal), nav2_core::PlannerException);
}

TYPED_TEST(GridSearchPlannerTest, MinimalOneCellCostmapDoesNotCrash)
{
  this->configurePlanner(CostmapOptions{1, 1, 1.0, 0.0, 0.0, false});
  auto pose = makePose(0.5, 0.5);
  EXPECT_NO_THROW(this->planner_.createPlan(pose, pose));
}

TYPED_TEST(GridSearchPlannerTest, ReturnsProvablyOptimalPathCostOnFreeCostmap)
{
  this->configurePlanner(CostmapOptions{10, 10, 0.1, 0.0, 0.0, false});
  auto * costmap = this->costmap_ros_->getCostmap();

  auto start = makePose(1.03, 1.07);
  auto goal = makePose(1.73, 1.31);

  unsigned int start_mx, start_my, goal_mx, goal_my;
  ASSERT_TRUE(
    costmap->worldToMap(
      start.pose.position.x, start.pose.position.y, start_mx, start_my));
  ASSERT_TRUE(
    costmap->worldToMap(
      goal.pose.position.x, goal.pose.position.y, goal_mx, goal_my));
  int dx = static_cast<int>(goal_mx) - static_cast<int>(start_mx);
  int dy = static_cast<int>(goal_my) - static_cast<int>(start_my);
  double expected_cost = amr_planner_plugins_test::octileDistance(dx, dy, costmap->getResolution());

  nav_msgs::msg::Path path;
  ASSERT_NO_THROW(path = this->planner_.createPlan(start, goal));
  double actual_cost = amr_planner_plugins_test::pathLength(path);

  EXPECT_NEAR(actual_cost, expected_cost, 1e-9);
}

}
