#pragma once

#include <algorithm>
#include <cmath>
#include <memory>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "vrobot_local_planner/msg/path.hpp"
#include "vrobot_local_planner/msg/planner_pose.hpp"

namespace vrobot_local_planner {
namespace geometry_utils {

/**
 * @brief Get a geometry_msgs Quaternion from a yaw angle
 * @param angle Yaw angle to generate a quaternion from
 * @return geometry_msgs Quaternion
 */
inline geometry_msgs::msg::Quaternion orientationAroundZAxis(double angle) {
  tf2::Quaternion q;
  q.setRPY(0, 0, angle); // void returning function
  return tf2::toMsg(q);
}

/**
 * @brief Get the euclidean distance between 2 geometry_msgs::Points
 * @param pos1 First point
 * @param pos1 Second point
 * @param is_3d True if a true L2 distance is desired (default false)
 * @return double L2 distance
 */
inline double euclidean_distance(const geometry_msgs::msg::Point &pos1,
                                 const geometry_msgs::msg::Point &pos2,
                                 const bool is_3d = false) {
  double dx = pos1.x - pos2.x;
  double dy = pos1.y - pos2.y;

  if (is_3d) {
    double dz = pos1.z - pos2.z;
    return std::hypot(dx, dy, dz);
  }

  return std::hypot(dx, dy);
}

/**
 * @brief Get the L2 distance between 2 geometry_msgs::Poses
 * @param pos1 First pose
 * @param pos1 Second pose
 * @param is_3d True if a true L2 distance is desired (default false)
 * @return double euclidean distance
 */
inline double euclidean_distance(const geometry_msgs::msg::Pose &pos1,
                                 const geometry_msgs::msg::Pose &pos2,
                                 const bool is_3d = false) {
  double dx = pos1.position.x - pos2.position.x;
  double dy = pos1.position.y - pos2.position.y;

  if (is_3d) {
    double dz = pos1.position.z - pos2.position.z;
    return std::hypot(dx, dy, dz);
  }

  return std::hypot(dx, dy);
}

/**
 * @brief Get the L2 distance between 2 geometry_msgs::PoseStamped
 * @param pos1 First pose
 * @param pos1 Second pose
 * @param is_3d True if a true L2 distance is desired (default false)
 * @return double L2 distance
 */
inline double euclidean_distance(const geometry_msgs::msg::PoseStamped &pos1,
                                 const geometry_msgs::msg::PoseStamped &pos2,
                                 const bool is_3d = false) {
  return euclidean_distance(pos1.pose, pos2.pose, is_3d);
}

/**
 * @brief Get the L2 distance between 2 geometry_msgs::Pose2D
 * @param pos1 First pose
 * @param pos1 Second pose
 * @return double L2 distance
 */
inline double euclidean_distance(const geometry_msgs::msg::Pose2D &pos1,
                                 const geometry_msgs::msg::Pose2D &pos2) {
  double dx = pos1.x - pos2.x;
  double dy = pos1.y - pos2.y;

  return std::hypot(dx, dy);
}

inline double
euclidean_distance(const vrobot_local_planner::msg::PlannerPose &pos1,
                   const vrobot_local_planner::msg::PlannerPose &pos2) {
  return euclidean_distance(pos1.pose, pos2.pose, true);
}

/**
 * Find element in iterator with the minimum calculated value
 */
template <typename Iter, typename Getter>
inline Iter min_by(Iter begin, Iter end, Getter getCompareVal) {
  if (begin == end) {
    return end;
  }
  auto lowest    = getCompareVal(*begin);
  Iter lowest_it = begin;
  for (Iter it = ++begin; it != end; ++it) {
    auto comp = getCompareVal(*it);
    if (comp < lowest) {
      lowest    = comp;
      lowest_it = it;
    }
  }
  return lowest_it;
}

/**
 * Find first element in iterator that is greater integrated distance than
 * comparevalue
 */
template <typename Iter, typename Getter>
inline Iter first_after_integrated_distance(Iter begin, Iter end,
                                            Getter getCompareVal) {
  if (begin == end) {
    return end;
  }
  Getter dist = 0.0;
  for (Iter it = begin; it != end - 1; it++) {
    dist += euclidean_distance(*it, *(it + 1));
    if (dist > getCompareVal) {
      return it + 1;
    }
  }
  return end;
}

/**
 * @brief Calculate the length of the provided path, starting at the provided
 * index
 * @param path Path containing the poses that are planned
 * @param start_index Optional argument specifying the starting index for
 * the calculation of path length. Provide this if you want to calculate length
 * of a subset of the path.
 * @return double Path length
 */
inline double calculate_path_length(const nav_msgs::msg::Path &path,
                                    size_t start_index = 0) {
  if (start_index + 1 >= path.poses.size()) {
    return 0.0;
  }
  double path_length = 0.0;
  for (size_t idx = start_index; idx < path.poses.size() - 1; ++idx) {
    path_length +=
        euclidean_distance(path.poses[idx].pose, path.poses[idx + 1].pose);
  }
  return path_length;
}

inline double calculate_path_length(const vrobot_local_planner::msg::Path &path,
                                    size_t start_index = 0) {
  if (start_index + 1 >= path.poses.size()) {
    return 0.0;
  }
  double path_length = 0.0;
  for (size_t idx = start_index; idx < path.poses.size() - 1; ++idx) {
    path_length +=
        euclidean_distance(path.poses[idx].pose, path.poses[idx + 1].pose);
  }
  return path_length;
}

} // namespace geometry_utils
} // namespace vrobot_local_planner
