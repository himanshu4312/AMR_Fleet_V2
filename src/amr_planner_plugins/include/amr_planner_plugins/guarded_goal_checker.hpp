#ifndef AMR_PLANNER_PLUGINS__GUARDED_GOAL_CHECKER_HPP_
#define AMR_PLANNER_PLUGINS__GUARDED_GOAL_CHECKER_HPP_

#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "nav2_controller/plugins/simple_goal_checker.hpp"

namespace amr_planner_plugins
{

class GuardedGoalChecker : public nav2_controller::SimpleGoalChecker
{
public:
  GuardedGoalChecker() = default;
  ~GuardedGoalChecker() override = default;

  bool isGoalReached(
    const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose,
    const geometry_msgs::msg::Twist & velocity) override;

private:

  std::atomic<unsigned long long> guarded_count_{0};

  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
};

}

#endif
