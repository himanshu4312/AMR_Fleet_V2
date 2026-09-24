#include "amr_planner_plugins/rrt_planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

#include "nav2_util/node_utils.hpp"
#include "amr_planner_plugins/costmap_cost_utils.hpp"

namespace amr_planner_plugins
{

void RRTPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  tf_ = tf;
  name_ = name;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  auto node = parent.lock();
  clock_ = node->get_clock();
  logger_ = node->get_logger();

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".allow_unknown", rclcpp::ParameterValue(true));
  node->get_parameter(name + ".allow_unknown", allow_unknown_);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".max_iterations", rclcpp::ParameterValue(5000));
  node->get_parameter(name + ".max_iterations", max_iterations_);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".step_size", rclcpp::ParameterValue(0.3));
  node->get_parameter(name + ".step_size", step_size_);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".goal_tolerance", rclcpp::ParameterValue(0.3));
  node->get_parameter(name + ".goal_tolerance", goal_tolerance_);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".goal_bias", rclcpp::ParameterValue(0.1));
  node->get_parameter(name + ".goal_bias", goal_bias_);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".planning_timeout", rclcpp::ParameterValue(1.0));
  node->get_parameter(name + ".planning_timeout", planning_timeout_);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".rng_seed", rclcpp::ParameterValue(-1));
  node->get_parameter(name + ".rng_seed", rng_seed_);

  if (rng_seed_ >= 0) {
    rng_.seed(static_cast<std::mt19937::result_type>(rng_seed_));
  } else {
    rng_.seed(std::random_device{}());
  }

  RCLCPP_INFO(
    logger_,
    "Configured RRTPlanner '%s' (max_iterations=%d, step_size=%.2f, goal_tolerance=%.2f, "
    "goal_bias=%.2f, planning_timeout=%.2fs)",
    name_.c_str(), max_iterations_, step_size_, goal_tolerance_, goal_bias_, planning_timeout_);
}

void RRTPlanner::cleanup() {RCLCPP_INFO(logger_, "Cleaning up RRTPlanner '%s'", name_.c_str());}
void RRTPlanner::activate() {RCLCPP_INFO(logger_, "Activating RRTPlanner '%s'", name_.c_str());}
void RRTPlanner::deactivate() {RCLCPP_INFO(logger_, "Deactivating RRTPlanner '%s'", name_.c_str());}

nav_msgs::msg::Path RRTPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{

  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));

  nav_msgs::msg::Path path;
  path.header.stamp = clock_->now();
  path.header.frame_id = global_frame_;

  unsigned int mx, my;
  if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, mx, my)) {
    throw nav2_core::PlannerException("Start pose is outside the costmap bounds.");
  }
  if (!costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, mx, my)) {
    throw nav2_core::PlannerException("Goal pose is outside the costmap bounds.");
  }
  if (!isStateValid(goal.pose.position.x, goal.pose.position.y)) {
    throw nav2_core::PlannerException("Goal pose is occupied by a lethal obstacle.");
  }

  const double min_x = costmap_->getOriginX();
  const double max_x = min_x + costmap_->getSizeInMetersX();
  const double min_y = costmap_->getOriginY();
  const double max_y = min_y + costmap_->getSizeInMetersY();

  std::uniform_real_distribution<double> x_dist(min_x, max_x);
  std::uniform_real_distribution<double> y_dist(min_y, max_y);
  std::uniform_real_distribution<double> unit_dist(0.0, 1.0);

  std::vector<TreeNode> tree;
  tree.push_back({start.pose.position.x, start.pose.position.y, -1});

  const auto search_start_time = std::chrono::steady_clock::now();
  int goal_node_index = -1;

  for (int iter = 0; iter < max_iterations_; ++iter) {
    double elapsed_s = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - search_start_time).count();
    if (elapsed_s > planning_timeout_) {
      throw nav2_core::PlannerException(
              "RRT planning exceeded planning_timeout (" +
              std::to_string(planning_timeout_) + "s) without finding a path.");
    }

    double sample_x, sample_y;
    if (unit_dist(rng_) < goal_bias_) {
      sample_x = goal.pose.position.x;
      sample_y = goal.pose.position.y;
    } else {
      sample_x = x_dist(rng_);
      sample_y = y_dist(rng_);
    }

    int nearest_idx = nearestNode(tree, sample_x, sample_y);
    const TreeNode & nearest = tree[nearest_idx];

    double dx = sample_x - nearest.x;
    double dy = sample_y - nearest.y;
    double dist = std::hypot(dx, dy);

    double new_x, new_y;
    if (dist <= step_size_) {
      new_x = sample_x;
      new_y = sample_y;
    } else {
      new_x = nearest.x + dx / dist * step_size_;
      new_y = nearest.y + dy / dist * step_size_;
    }

    if (!isStateValid(new_x, new_y) || !isSegmentFree(nearest.x, nearest.y, new_x, new_y)) {
      continue;
    }

    tree.push_back({new_x, new_y, nearest_idx});
    int new_idx = static_cast<int>(tree.size()) - 1;

    double dist_to_goal = std::hypot(goal.pose.position.x - new_x, goal.pose.position.y - new_y);
    if (dist_to_goal <= goal_tolerance_ &&
      isSegmentFree(new_x, new_y, goal.pose.position.x, goal.pose.position.y))
    {
      tree.push_back({goal.pose.position.x, goal.pose.position.y, new_idx});
      goal_node_index = static_cast<int>(tree.size()) - 1;
      break;
    }
  }

  if (goal_node_index < 0) {
    throw nav2_core::PlannerException(
            "RRT failed to find a path within max_iterations (" +
            std::to_string(max_iterations_) + ").");
  }

  std::vector<TreeNode> result;
  for (int idx = goal_node_index; idx != -1; idx = tree[idx].parent) {
    result.push_back(tree[idx]);
  }
  std::reverse(result.begin(), result.end());

  path.poses.reserve(result.size());
  for (const auto & node : result) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = node.x;
    pose.pose.position.y = node.y;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }

  if (!path.poses.empty()) {
    path.poses.back().pose.orientation = goal.pose.orientation;
  }

  return path;
}

bool RRTPlanner::isStateValid(double x, double y) const
{
  unsigned int mx, my;
  if (!costmap_->worldToMap(x, y, mx, my)) {
    return false;
  }
  return !isLethalCost(costmap_->getCost(mx, my), allow_unknown_);
}

bool RRTPlanner::isSegmentFree(double x0, double y0, double x1, double y1) const
{
  double dist = std::hypot(x1 - x0, y1 - y0);

  double check_step = costmap_->getResolution() * 0.5;
  int steps = std::max(1, static_cast<int>(std::ceil(dist / check_step)));

  for (int i = 0; i <= steps; ++i) {
    double t = static_cast<double>(i) / static_cast<double>(steps);
    double x = x0 + t * (x1 - x0);
    double y = y0 + t * (y1 - y0);
    if (!isStateValid(x, y)) {
      return false;
    }
  }
  return true;
}

int RRTPlanner::nearestNode(const std::vector<TreeNode> & tree, double x, double y) const
{
  int best_idx = 0;
  double best_dist_sq = std::numeric_limits<double>::max();
  for (std::size_t i = 0; i < tree.size(); ++i) {
    double dx = tree[i].x - x;
    double dy = tree[i].y - y;
    double dist_sq = dx * dx + dy * dy;
    if (dist_sq < best_dist_sq) {
      best_dist_sq = dist_sq;
      best_idx = static_cast<int>(i);
    }
  }
  return best_idx;
}

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(amr_planner_plugins::RRTPlanner, nav2_core::GlobalPlanner)
