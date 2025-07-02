#pragma once

#include <chrono>
#include <cmath>
#include <deque>
#include <memory>
#include <mutex>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "vrobot_local_planner/utils/node_utils.hpp"

namespace vrobot_local_planner {

/**
 * @class OdomSmoother
 * Wrapper for getting smooth odometry readings using a simple moving avergae.
 * Subscribes to the topic with a mutex.
 */
class OdomSmoother {
public:
  /**
   * @brief Constructor that subscribes to an Odometry topic
   * @param parent NodeHandle for creating subscriber
   * @param filter_duration Duration for odom history (seconds)
   * @param odom_topic Topic on which odometry should be received
   */
  explicit OdomSmoother(const rclcpp::Node::WeakPtr &parent,
                        double                       filter_duration = 0.3,
                        const std::string           &odom_topic      = "odom");

  /**
   * @brief Get twist msg from smoother
   * @return twist Twist msg
   */
  inline geometry_msgs::msg::Twist getTwist() { return vel_smooth_.twist; }

  /**
   * @brief Get twist stamped msg from smoother
   * @return twist TwistStamped msg
   */
  inline geometry_msgs::msg::TwistStamped getTwistStamped() {
    return vel_smooth_;
  }

protected:
  /**
   * @brief Callback of odometry subscriber to process
   * @param msg Odometry msg to smooth
   */
  void odomCallback(nav_msgs::msg::Odometry::SharedPtr msg);

  /**
   * @brief Update internal state of the smoother after getting new data
   */
  void updateState();

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  nav_msgs::msg::Odometry                                  odom_cumulate_;
  geometry_msgs::msg::TwistStamped                         vel_smooth_;
  std::mutex                                               odom_mutex_;

  rclcpp::Duration                    odom_history_duration_;
  std::deque<nav_msgs::msg::Odometry> odom_history_;
};

} // namespace vrobot_local_planner
