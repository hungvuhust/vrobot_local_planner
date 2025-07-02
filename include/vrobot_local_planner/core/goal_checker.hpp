#pragma once

#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace vrobot_local_planner {

class GoalChecker {
public:
  typedef std::shared_ptr<vrobot_local_planner::GoalChecker> Ptr;

  virtual ~GoalChecker() {}

  virtual void initialize(
      const rclcpp::Node::WeakPtr &parent, const std::string &plugin_name,
      const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) = 0;

  virtual void reset() = 0;

  virtual bool isGoalReached(const geometry_msgs::msg::Pose  &query_pose,
                             const geometry_msgs::msg::Pose  &goal_pose,
                             const geometry_msgs::msg::Twist &velocity) = 0;

  virtual bool getTolerances(geometry_msgs::msg::Pose  &pose_tolerance,
                             geometry_msgs::msg::Twist &vel_tolerance) = 0;
};

} // namespace vrobot_local_planner