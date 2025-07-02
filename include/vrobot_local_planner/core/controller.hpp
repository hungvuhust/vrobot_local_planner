#pragma once

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "vrobot_local_planner/core/goal_checker.hpp"
#include "vrobot_local_planner/msg/path.hpp"
#include "vrobot_local_planner/msg/planner_pose.hpp"

namespace vrobot_local_planner {

class Controller {
public:
  using Ptr = std::shared_ptr<vrobot_local_planner::Controller>;

  virtual ~Controller() {}

  virtual void configure(const rclcpp::Node::WeakPtr &, std::string name,
                         std::shared_ptr<tf2_ros::Buffer>,
                         std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) = 0;

  virtual void setPlan(const vrobot_local_planner::msg::Path &path) = 0;

  virtual geometry_msgs::msg::TwistStamped
  computeVelocityCommands(const vrobot_local_planner::msg::PlannerPose &pose,
                          const geometry_msgs::msg::Twist   &velocity,
                          vrobot_local_planner::GoalChecker *goal_checker) = 0;

  virtual void setSpeedLimit(const double &speed_limit,
                             const bool   &percentage) = 0;
};

} // namespace vrobot_local_planner