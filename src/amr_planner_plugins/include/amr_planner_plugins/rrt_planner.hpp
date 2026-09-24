#ifndef AMR_PLANNER_PLUGINS__RRT_PLANNER_HPP_
#define AMR_PLANNER_PLUGINS__RRT_PLANNER_HPP_

#include <memory>
#include <random>
#include <string>
#include <vector>

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

class RRTPlanner : public nav2_core::GlobalPlanner
{
public:
  RRTPlanner() = default;
  ~RRTPlanner() override = default;

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

private:
  struct TreeNode
  {
    double x;
    double y;
    int parent;
  };

  bool isStateValid(double x, double y) const;
  bool isSegmentFree(double x0, double y0, double x1, double y1) const;

  int nearestNode(const std::vector<TreeNode> & tree, double x, double y) const;

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_{rclcpp::get_logger("RRTPlanner")};
  std::string global_frame_, name_;

  bool allow_unknown_{true};
  int max_iterations_{5000};
  double step_size_{0.3};
  double goal_tolerance_{0.3};
  double goal_bias_{0.1};
  double planning_timeout_{1.0};

  int rng_seed_{-1};

  mutable std::mt19937 rng_;
};

}

#endif
