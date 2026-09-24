#include "amr_planner_plugins/guarded_goal_checker.hpp"

#include <sstream>
#include <string>

namespace amr_planner_plugins
{

namespace
{
std::string poseToString(const geometry_msgs::msg::Pose & pose)
{
  std::ostringstream oss;
  oss << "position=(" << pose.position.x << ", " << pose.position.y << ", "
      << pose.position.z << ") orientation=(" << pose.orientation.x << ", "
      << pose.orientation.y << ", " << pose.orientation.z << ", " << pose.orientation.w << ")";
  return oss.str();
}
}

bool GuardedGoalChecker::isGoalReached(
  const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose,
  const geometry_msgs::msg::Twist & velocity)
{
  const auto & p = goal_pose.position;
  const auto & q = goal_pose.orientation;

  const bool is_default_pose =
    (p.x == 0.0 && p.y == 0.0 && p.z == 0.0) &&
    (q.x == 0.0 && q.y == 0.0 && q.z == 0.0 && q.w == 1.0);

  if (is_default_pose) {
    const unsigned long long count = ++guarded_count_;

    RCLCPP_WARN_THROTTLE(
      rclcpp::get_logger(plugin_name_), steady_clock_, 5000,
      "GuardedGoalChecker [%s]: goal_pose was the exact default-constructed "
      "pose (position (0,0,0), orientation (0,0,0,1)) - this is "
      "nav2_controller's known defect where a failed map->odom TF "
      "transform's failure return value is ignored by isGoalReached()'s "
      "caller, which then passes in a never-written-to default "
      "PoseStamped instead of the real goal. Treating as 'not reached yet' "
      "instead of falsely succeeding. Guarded %llu time(s) since startup. "
      "query_pose: %s. goal_pose: %s.",
      plugin_name_.c_str(), count,
      poseToString(query_pose).c_str(), poseToString(goal_pose).c_str());
    return false;
  }
  return SimpleGoalChecker::isGoalReached(query_pose, goal_pose, velocity);
}

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(amr_planner_plugins::GuardedGoalChecker, nav2_core::GoalChecker)
