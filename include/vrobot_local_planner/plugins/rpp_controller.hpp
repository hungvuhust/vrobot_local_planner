#pragma once

#include "vrobot_local_planner/core/controller.hpp"
#include <algorithm>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "rclcpp/rclcpp.hpp"
#include "vrobot_local_planner/utils/geometry_utils.hpp"

namespace vrobot_local_planner {

class RPPController : public Controller {
public:
  RPPController()           = default;
  ~RPPController() override = default;

  void configure(const rclcpp::Node::WeakPtr &, std::string name,
                 std::shared_ptr<tf2_ros::Buffer>,
                 std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override;

  void setPlan(const vrobot_local_planner::msg::Path &path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
      const vrobot_local_planner::msg::PlannerPose &pose,
      const geometry_msgs::msg::Twist              &velocity,
      vrobot_local_planner::GoalChecker            *goal_checker) override;

  void setSpeedLimit(const double &speed_limit,
                     const bool   &percentage) override;

protected:
  vrobot_local_planner::msg::Path
  transformGlobalPlan(const vrobot_local_planner::msg::PlannerPose &pose);

  bool transformPose(const std::string                             frame,
                     const vrobot_local_planner::msg::PlannerPose &in_pose,
                     vrobot_local_planner::msg::PlannerPose &out_pose) const;

  double getLookAheadDistance(const geometry_msgs::msg::Twist &);

  std::unique_ptr<geometry_msgs::msg::PointStamped>
  createCarrotMsg(const vrobot_local_planner::msg::PlannerPose &carrot_pose);

  bool
  shouldRotateToPath(const vrobot_local_planner::msg::PlannerPose &carrot_pose,
                     double &angle_to_path);

  bool shouldRotateToGoalHeading(
      const vrobot_local_planner::msg::PlannerPose &carrot_pose);

  void rotateToHeading(double &linear_vel, double &angular_vel,
                       const double                    &angle_to_path,
                       const geometry_msgs::msg::Twist &curr_speed);

  bool isCollisionImminent(const vrobot_local_planner::msg::PlannerPose &,
                           const double &, const double &, const double &);

  bool inCollision(const double &x, const double &y, const double &theta);

  double costAtPose(const double &x, const double &y);

  double approachVelocityScalingFactor(
      const vrobot_local_planner::msg::Path &path) const;

  void applyApproachVelocityScaling(const vrobot_local_planner::msg::Path &path,
                                    double &linear_vel) const;

  void applyConstraints(const double                          &curvature,
                        const geometry_msgs::msg::Twist       &speed,
                        const double                          &pose_cost,
                        const vrobot_local_planner::msg::Path &path,
                        double &linear_vel, double &sign);

  static geometry_msgs::msg::Point
  circleSegmentIntersection(const geometry_msgs::msg::Point &p1,
                            const geometry_msgs::msg::Point &p2, double r);

  vrobot_local_planner::msg::PlannerPose
  getLookAheadPoint(const double &, const vrobot_local_planner::msg::Path &);

  double findVelocitySignChange(
      const vrobot_local_planner::msg::Path &transformed_plan);

  double getCostmapMaxExtent() const;

  rclcpp::Node::WeakPtr                          node_;
  std::shared_ptr<tf2_ros::Buffer>               tf_;
  std::string                                    plugin_name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D                    *costmap_;
  rclcpp::Logger logger_{rclcpp::get_logger("RegulatedPurePursuitController")};
  rclcpp::Clock::SharedPtr clock_;

  double        desired_linear_vel_, base_desired_linear_vel_;
  double        lookahead_dist_;
  double        rotate_to_heading_angular_vel_;
  double        max_lookahead_dist_;
  double        min_lookahead_dist_;
  double        lookahead_time_;
  bool          use_velocity_scaled_lookahead_dist_;
  tf2::Duration transform_tolerance_;
  double        min_approach_linear_velocity_;
  double        approach_velocity_scaling_dist_;
  double        control_duration_;
  double        max_allowed_time_to_collision_up_to_carrot_;
  bool          use_collision_detection_;
  bool          use_regulated_linear_velocity_scaling_;
  bool          use_cost_regulated_linear_velocity_scaling_;
  double        cost_scaling_dist_;
  double        cost_scaling_gain_;
  double        inflation_cost_scaling_factor_;
  double        regulated_linear_scaling_min_radius_;
  double        regulated_linear_scaling_min_speed_;
  bool          use_rotate_to_heading_;
  double        max_angular_accel_;
  double        rotate_to_heading_min_angle_;
  double        goal_dist_tol_;
  bool          allow_reversing_;
  double        max_robot_pose_search_dist_;
  bool          use_interpolation_;

  vrobot_local_planner::msg::Path global_plan_;
  std::shared_ptr<rclcpp::Publisher<vrobot_local_planner::msg::Path>>
      global_path_pub_;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PointStamped>>
                                                          carrot_pub_;
  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Path>> carrot_arc_pub_;
  std::unique_ptr<
      nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>
      collision_checker_;

  std::mutex mutex_;
};

} // namespace vrobot_local_planner