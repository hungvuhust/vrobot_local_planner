#pragma once

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav_2d_utils/conversions.hpp"
#include "nav_2d_utils/odom_subscriber.hpp"
#include "nav_2d_utils/tf_help.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "vrobot_local_planner/action/follow_path.hpp"
#include "vrobot_local_planner/action/v_follow_path.hpp"
#include "vrobot_local_planner/core/controller.hpp"
#include "vrobot_local_planner/core/goal_checker.hpp"
#include "vrobot_local_planner/msg/path.hpp"
#include "vrobot_local_planner/msg/planner_pose.hpp"
#include "vrobot_local_planner/plugins/rpp_controller.hpp"
#include "vrobot_local_planner/plugins/simple_goal_checker.hpp"
#include "vrobot_local_planner/utils/geometry_utils.hpp"
#include "vrobot_local_planner/utils/node_utils.hpp"
#include "vrobot_local_planner/utils/odometry_utils.hpp"
#include "vrobot_local_planner/utils/simple_action_server.hpp"
#include <chrono>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

namespace vrobot_local_planner {

class ControllerServer : public rclcpp::Node {
public:
  using ControllerMap  = std::unordered_map<std::string, Controller::Ptr>;
  using GoalCheckerMap = std::unordered_map<std::string, GoalChecker::Ptr>;

  explicit ControllerServer(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~ControllerServer();
  void on_configure();

protected:
  using Action       = vrobot_local_planner::action::VFollowPath;
  using ActionServer = vrobot_local_planner::SimpleActionServer<Action>;
  std::unique_ptr<ActionServer> action_server_;

  bool init_parameters();
  bool init_publishers_subscribers();
  bool init_controllers();
  bool init_actions();
  bool init_costmap();

  void computeControl();

  void setPlannerPath(const vrobot_local_planner::msg::Path &path);

  void computeAndPublishVelocity();

  void updateGlobalPath();

  void publishVelocity(const geometry_msgs::msg::TwistStamped &velocity);

  void publishZeroVelocity();

  bool isGoalReached();

  bool findGoalCheckerId(const std::string &c_name,
                         std::string       &current_goal_checker);

  bool findControllerId(const std::string &c_name,
                        std::string       &current_controller);

  bool getRobotPose(vrobot_local_planner::msg::PlannerPose &pose);

  double getThresholdedVelocity(double velocity, double threshold) {
    return (std::abs(velocity) > threshold) ? velocity : 0.0;
  }

  geometry_msgs::msg::Twist
  getThresholdedTwist(const geometry_msgs::msg::Twist &twist) {
    geometry_msgs::msg::Twist twist_thresh;
    twist_thresh.linear.x =
        getThresholdedVelocity(twist.linear.x, min_x_velocity_threshold_);
    twist_thresh.linear.y =
        getThresholdedVelocity(twist.linear.y, min_y_velocity_threshold_);
    twist_thresh.angular.z =
        getThresholdedVelocity(twist.angular.z, min_theta_velocity_threshold_);
    return twist_thresh;
  }

  // The controller needs a costmap node
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS>    costmap_ros_;
  std::unique_ptr<vrobot_local_planner::NodeThread> costmap_thread_;

  // Publishers and subscribers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;
  std::shared_ptr<vrobot_local_planner::OdomSmoother>     odom_sub_;

  // Goal Checker Plugin
  pluginlib::ClassLoader<vrobot_local_planner::GoalChecker>
                           goal_checker_loader_;
  GoalCheckerMap           goal_checkers_;
  std::vector<std::string> default_goal_checker_ids_;
  std::vector<std::string> default_goal_checker_types_;
  std::vector<std::string> goal_checker_ids_;
  std::vector<std::string> goal_checker_types_;
  std::string              goal_checker_ids_concat_, current_goal_checker_;

  // Controller Plugins
  pluginlib::ClassLoader<vrobot_local_planner::Controller> lp_loader_;
  ControllerMap                                            controllers_;
  std::vector<std::string>                                 default_ids_;
  std::vector<std::string>                                 default_types_;
  std::vector<std::string>                                 controller_ids_;
  std::vector<std::string>                                 controller_types_;
  std::string controller_ids_concat_, current_controller_;

  double controller_frequency_;
  double min_x_velocity_threshold_;
  double min_y_velocity_threshold_;
  double min_theta_velocity_threshold_;

  double failure_tolerance_;

  // Whether we've published the single controller warning yet
  vrobot_local_planner::msg::PlannerPose end_pose_;

  // Last time the controller generated a valid command
  rclcpp::Time last_valid_cmd_time_;

  // Current path container
  vrobot_local_planner::msg::Path current_path_;
};
} // namespace vrobot_local_planner