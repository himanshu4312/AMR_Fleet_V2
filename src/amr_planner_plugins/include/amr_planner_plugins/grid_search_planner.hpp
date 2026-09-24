#ifndef AMR_PLANNER_PLUGINS__GRID_SEARCH_PLANNER_HPP_
#define AMR_PLANNER_PLUGINS__GRID_SEARCH_PLANNER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <unordered_map>
#include <queue>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"

namespace amr_planner_plugins
{

class GridSearchPlanner : public nav2_core::GlobalPlanner
{
public:
  GridSearchPlanner() = default;
  ~GridSearchPlanner() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

protected:
  struct CellIndex
  {
    int x;
    int y;
    bool operator==(const CellIndex & other) const {return x == other.x && y == other.y;}
  };

  struct CellIndexHash
  {
    std::size_t operator()(const CellIndex & idx) const
    {
      return (static_cast<std::size_t>(idx.x) << 16) ^ static_cast<std::size_t>(idx.y);
    }
  };

  struct OpenSetEntry
  {
    CellIndex index;
    double f_cost;
  };

  struct OpenSetCompare
  {
    bool operator()(const OpenSetEntry & a, const OpenSetEntry & b) const
    {
      return a.f_cost > b.f_cost;
    }
  };

  virtual double heuristic(const CellIndex & a, const CellIndex & b) const = 0;

  virtual std::string plannerTypeName() const = 0;

  bool searchPath(
    const CellIndex & start_cell, const CellIndex & goal_cell,
    std::vector<CellIndex> & result_path);

  double traversalCost(const CellIndex & from, const CellIndex & to) const;
  std::vector<CellIndex> neighbors(const CellIndex & idx) const;
  bool inBounds(const CellIndex & idx) const;
  bool isLethal(const CellIndex & idx) const;

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_{rclcpp::get_logger("GridSearchPlanner")};
  std::string global_frame_, name_;

  bool allow_unknown_{true};
  double cost_weight_{0.8};
};

}

#endif
