#include "vrobot_local_planner/plugins/stopped_goal_checker.hpp"
#include "angles/angles.h"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <limits>
#include <memory>
#include <string>
#include <vector>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "tf2/utils.h"
#pragma GCC diagnostic pop

namespace vrobot_local_planner {

StoppedGoalChecker::StoppedGoalChecker()
    : xy_goal_tolerance_(0.25), yaw_goal_tolerance_(0.25), stateful_(true),
      check_xy_(true), xy_goal_tolerance_sq_(0.0625) {}

void StoppedGoalChecker::initialize(
    const rclcpp::Node::WeakPtr &parent, const std::string &plugin_name,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> /*costmap_ros*/) {
  plugin_name_ = plugin_name;
  auto node    = parent.lock();

  node->declare_parameter(plugin_name + ".xy_goal_tolerance", 0.25);
  node->declare_parameter(plugin_name + ".yaw_goal_tolerance", 0.25);
  node->declare_parameter(plugin_name + ".stateful", true);

  node->get_parameter(plugin_name + ".xy_goal_tolerance", xy_goal_tolerance_);
  node->get_parameter(plugin_name + ".yaw_goal_tolerance", yaw_goal_tolerance_);
  node->get_parameter(plugin_name + ".stateful", stateful_);

  xy_goal_tolerance_sq_ = xy_goal_tolerance_ * xy_goal_tolerance_;
}

void StoppedGoalChecker::reset() { check_xy_ = true; }

bool StoppedGoalChecker::isGoalReached(
    const geometry_msgs::msg::Pose &query_pose,
    const geometry_msgs::msg::Pose &goal_pose,
    const geometry_msgs::msg::Twist &) {
  if (check_xy_) {
    double dx = query_pose.position.x - goal_pose.position.x,
           dy = query_pose.position.y - goal_pose.position.y;
    if (dx * dx + dy * dy > xy_goal_tolerance_sq_) {
      return false;
    }
    // We are within the window
    // If we are stateful, change the state.
    if (stateful_) {
      check_xy_ = false;
    }
  }
  double dyaw = angles::shortest_angular_distance(
      tf2::getYaw(query_pose.orientation), tf2::getYaw(goal_pose.orientation));
  return fabs(dyaw) < yaw_goal_tolerance_;
}

bool StoppedGoalChecker::getTolerances(
    geometry_msgs::msg::Pose  &pose_tolerance,
    geometry_msgs::msg::Twist &vel_tolerance) {
  double invalid_field = std::numeric_limits<double>::lowest();

  pose_tolerance.position.x = xy_goal_tolerance_;
  pose_tolerance.position.y = xy_goal_tolerance_;
  pose_tolerance.position.z = invalid_field;
  pose_tolerance.orientation =
      nav2_util::geometry_utils::orientationAroundZAxis(yaw_goal_tolerance_);

  vel_tolerance.linear.x = invalid_field;
  vel_tolerance.linear.y = invalid_field;
  vel_tolerance.linear.z = invalid_field;

  vel_tolerance.angular.x = invalid_field;
  vel_tolerance.angular.y = invalid_field;
  vel_tolerance.angular.z = invalid_field;

  return true;
}

} // namespace vrobot_local_planner
#include "pluginlib/class_list_macros.hpp"
#include "pluginlib/class_loader.hpp"
PLUGINLIB_EXPORT_CLASS(vrobot_local_planner::StoppedGoalChecker,
                       vrobot_local_planner::GoalChecker)