#pragma once

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "vrobot_local_planner/core/goal_checker.hpp"

namespace vrobot_local_planner {

class SimpleGoalChecker : public vrobot_local_planner::GoalChecker {
public:
  SimpleGoalChecker();
  // Standard GoalChecker Interface
  void initialize(const rclcpp::Node::WeakPtr &parent,
                  const std::string           &plugin_name,
                  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>
                      costmap_ros) override;
  void reset() override;
  bool isGoalReached(const geometry_msgs::msg::Pose  &query_pose,
                     const geometry_msgs::msg::Pose  &goal_pose,
                     const geometry_msgs::msg::Twist &velocity) override;
  bool getTolerances(geometry_msgs::msg::Pose  &pose_tolerance,
                     geometry_msgs::msg::Twist &vel_tolerance) override;

protected:
  double xy_goal_tolerance_, yaw_goal_tolerance_;
  bool   stateful_, check_xy_;
  // Cached squared xy_goal_tolerance_
  double xy_goal_tolerance_sq_;

  std::string plugin_name_;
};

} // namespace vrobot_local_planner