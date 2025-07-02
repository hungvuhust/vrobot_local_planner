#include "vrobot_local_planner/controller_server.hpp"
#include <memory>

using namespace std::chrono_literals;
using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace vrobot_local_planner {

ControllerServer::ControllerServer(const rclcpp::NodeOptions &options)
    : rclcpp::Node("controller_server", "", options),
      goal_checker_loader_("vrobot_local_planner",
                           "vrobot_local_planner::GoalChecker"),
      default_goal_checker_ids_{"simple_goal_checker"},
      default_goal_checker_types_{"vrobot_local_planner::SimpleGoalChecker"},
      lp_loader_("vrobot_local_planner", "vrobot_local_planner::Controller"),
      default_ids_{"FollowPath"}, default_types_{
                                      "vrobot_local_planner::RPPController"} {
  RCLCPP_INFO(get_logger(), "Creating controller server");
  // Initialize parameters
  init_parameters();
}

ControllerServer::~ControllerServer() { costmap_thread_.reset(); }

void ControllerServer::on_configure() {
  try {
    // Initialize costmap
    if (!init_costmap()) {
      RCLCPP_ERROR(get_logger(), "Failed to initialize costmap");
      throw std::runtime_error("Failed to initialize costmap");
    }
    // Initialize controllers
    if (!init_controllers()) {
      RCLCPP_ERROR(get_logger(), "Failed to initialize controllers");
      throw std::runtime_error("Failed to initialize controllers");
    }
    // Initialize actions
    if (!init_actions()) {
      RCLCPP_ERROR(get_logger(), "Failed to initialize actions");
      throw std::runtime_error("Failed to initialize actions");
    }
    // Initialize publishers and subscribers
    if (!init_publishers_subscribers()) {
      RCLCPP_ERROR(get_logger(),
                   "Failed to initialize publishers and subscribers");
      throw std::runtime_error(
          "Failed to initialize publishers and subscribers");
    }

    costmap_ros_->activate();
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_logger(), "Failed to configure controller server: %s",
                 e.what());
    throw;
  }
}

bool ControllerServer::init_parameters() {
  this->declare_parameter("controller_frequency", 20.0);

  this->declare_parameter("min_x_velocity_threshold",
                          rclcpp::ParameterValue(0.0001));
  this->declare_parameter("min_y_velocity_threshold",
                          rclcpp::ParameterValue(0.0001));
  this->declare_parameter("min_theta_velocity_threshold",
                          rclcpp::ParameterValue(0.0001));

  this->declare_parameter("failure_tolerance", rclcpp::ParameterValue(0.0));
  this->declare_parameter("goal_checker_plugins", default_goal_checker_ids_);
  this->declare_parameter("controller_plugins", default_ids_);

  this->get_parameter("controller_frequency", controller_frequency_);
  this->get_parameter("min_x_velocity_threshold", min_x_velocity_threshold_);
  this->get_parameter("min_y_velocity_threshold", min_y_velocity_threshold_);
  this->get_parameter("min_theta_velocity_threshold",
                      min_theta_velocity_threshold_);
  this->get_parameter("failure_tolerance", failure_tolerance_);

  RCLCPP_INFO(get_logger(), "Controller frequency set to %.4fHz",
              controller_frequency_);

  return true;
}

bool ControllerServer::init_publishers_subscribers() {
  RCLCPP_INFO(get_logger(), "Initializing publishers and subscribers");
  vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
  odom_sub_      = std::make_shared<vrobot_local_planner::OdomSmoother>(
      shared_from_this(), 0.3, "odom");

  return true;
}

bool ControllerServer::init_controllers() {
  RCLCPP_INFO(get_logger(), "Initializing controllers");
  auto node = shared_from_this();

  get_parameter("goal_checker_plugins", goal_checker_ids_);
  RCLCPP_INFO(get_logger(), "goal_checker_ids_: %zu", goal_checker_ids_.size());

  if (goal_checker_ids_.size() > 0) {
    for (size_t i = 0; i < goal_checker_ids_.size(); ++i) {
      RCLCPP_INFO(get_logger(), "goal_checker_ids_[%zu]: %s", i,
                  goal_checker_ids_[i].c_str());
      vrobot_local_planner::declare_parameter_if_not_declared(
          node, goal_checker_ids_[i] + ".plugin",
          rclcpp::ParameterValue(default_goal_checker_types_[0]));
    }
  }

  get_parameter("controller_plugins", controller_ids_);
  RCLCPP_INFO(get_logger(), "controller_ids_: %zu", controller_ids_.size());

  if (controller_ids_ == default_ids_) {
    for (size_t i = 0; i < default_ids_.size(); ++i) {
      vrobot_local_planner::declare_parameter_if_not_declared(
          node, default_ids_[i] + ".plugin",
          rclcpp::ParameterValue(default_types_[i]));
    }
  }

  controller_types_.resize(controller_ids_.size());
  goal_checker_types_.resize(goal_checker_ids_.size());

  for (size_t i = 0; i != controller_ids_.size(); i++) {
    try {
      controller_types_[i] =
          vrobot_local_planner::get_plugin_type_param(node, controller_ids_[i]);
      vrobot_local_planner::Controller::Ptr controller =
          lp_loader_.createUniqueInstance(controller_types_[i]);
      RCLCPP_INFO(get_logger(), "Created controller : %s of type %s",
                  controller_ids_[i].c_str(), controller_types_[i].c_str());
      controller->configure(node, controller_ids_[i],
                            costmap_ros_->getTfBuffer(), costmap_ros_);
      controllers_.insert({controller_ids_[i], controller});
    } catch (const pluginlib::PluginlibException &ex) {
      RCLCPP_FATAL(get_logger(), "Failed to create controller. Exception: %s",
                   ex.what());
      return false;
    }
  }

  for (size_t i = 0; i != controller_ids_.size(); i++) {
    controller_ids_concat_ += controller_ids_[i] + std::string(" ");
  }

  RCLCPP_INFO(get_logger(), "Controller Server has %s controllers available.",
              controller_ids_concat_.c_str());

  RCLCPP_INFO(get_logger(), "Initializing goal checker");
  for (size_t i = 0; i != goal_checker_ids_.size(); i++) {
    try {
      goal_checker_types_[i] = vrobot_local_planner::get_plugin_type_param(
          node, goal_checker_ids_[i]);
      vrobot_local_planner::GoalChecker::Ptr goal_checker =
          goal_checker_loader_.createUniqueInstance(goal_checker_types_[i]);
      RCLCPP_INFO(get_logger(), "Created goal checker : %s of type %s",
                  goal_checker_ids_[i].c_str(), goal_checker_types_[i].c_str());
      goal_checker->initialize(node, goal_checker_ids_[i], costmap_ros_);
      goal_checkers_.insert({goal_checker_ids_[i], goal_checker});
    } catch (const pluginlib::PluginlibException &ex) {
      RCLCPP_FATAL(get_logger(), "Failed to create goal checker. Exception: %s",
                   ex.what());
      return false;
    }
  }

  for (size_t i = 0; i != goal_checker_ids_.size(); i++) {
    goal_checker_ids_concat_ += goal_checker_ids_[i] + std::string(" ");
  }

  RCLCPP_INFO(get_logger(),
              "Goal checker Server has %s goal checkers available.",
              goal_checker_ids_concat_.c_str());

  return true;
}

bool ControllerServer::init_actions() {
  action_server_ = std::make_unique<ActionServer>(
      this, "v_follow_path", std::bind(&ControllerServer::computeControl, this),
      nullptr, std::chrono::milliseconds(500), true);

  action_server_->activate();

  return true;
}

bool ControllerServer::init_costmap() {
  // The costmap node is used in the implementation of the controller
  costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
      "local_costmap", std::string{get_namespace()}, "local_costmap");
  costmap_ros_->configure();
  // Launch a thread to run the costmap node
  costmap_thread_ =
      std::make_unique<vrobot_local_planner::NodeThread>(costmap_ros_);
  return true;
}

bool ControllerServer::findControllerId(const std::string &c_name,
                                        std::string       &current_controller) {
  if (controllers_.find(c_name) == controllers_.end()) {
    if (controllers_.size() == 1 && c_name.empty()) {
      RCLCPP_WARN_ONCE(get_logger(),
                       "No controller was specified in action call."
                       " Server will use only plugin loaded %s. "
                       "This warning will appear once.",
                       controller_ids_concat_.c_str());
      current_controller = controllers_.begin()->first;
    } else {
      RCLCPP_ERROR(get_logger(),
                   "FollowPath called with controller name %s, "
                   "which does not exist. Available controllers are: %s.",
                   c_name.c_str(), controller_ids_concat_.c_str());
      return false;
    }
  } else {
    RCLCPP_DEBUG(get_logger(), "Selected controller: %s.", c_name.c_str());
    current_controller = c_name;
  }

  return true;
}

bool ControllerServer::findGoalCheckerId(const std::string &c_name,
                                         std::string &current_goal_checker) {
  if (goal_checkers_.find(c_name) == goal_checkers_.end()) {
    if (goal_checkers_.size() == 1 && c_name.empty()) {
      RCLCPP_WARN_ONCE(
          get_logger(),
          "No goal checker was specified in parameter 'current_goal_checker'."
          " Server will use only plugin loaded %s. "
          "This warning will appear once.",
          goal_checker_ids_concat_.c_str());
      current_goal_checker = goal_checkers_.begin()->first;
    } else {
      RCLCPP_ERROR(get_logger(),
                   "FollowPath called with goal_checker name %s in parameter"
                   " 'current_goal_checker', which does not exist. Available "
                   "goal checkers are: %s.",
                   c_name.c_str(), goal_checker_ids_concat_.c_str());
      return false;
    }
  } else {
    RCLCPP_DEBUG(get_logger(), "Selected goal checker: %s.", c_name.c_str());
    current_goal_checker = c_name;
  }

  return true;
}

void ControllerServer::computeControl() {

  RCLCPP_INFO(get_logger(), "Received a goal, begin computing control effort.");

  try {
    std::string c_name = action_server_->get_current_goal()->controller_id;
    std::string current_controller;
    if (findControllerId(c_name, current_controller)) {
      current_controller_ = current_controller;
    } else {
      action_server_->terminate_current();
      return;
    }

    std::string gc_name = action_server_->get_current_goal()->goal_checker_id;
    std::string current_goal_checker;
    if (findGoalCheckerId(gc_name, current_goal_checker)) {
      current_goal_checker_ = current_goal_checker;
    } else {
      action_server_->terminate_current();
      return;
    }

    setPlannerPath(action_server_->get_current_goal()->path);

    rclcpp::WallRate loop_rate(controller_frequency_);
    while (rclcpp::ok()) {
      if (action_server_ == nullptr || !action_server_->is_server_active()) {
        RCLCPP_DEBUG(get_logger(),
                     "Action server unavailable or inactive. Stopping.");
        return;
      }

      if (action_server_->is_cancel_requested()) {
        RCLCPP_INFO(get_logger(), "Goal was canceled. Stopping the robot.");
        action_server_->terminate_all();
        publishZeroVelocity();
        return;
      }

      // Don't compute a trajectory until costmap is valid (after clear costmap)
      rclcpp::Rate r(100);
      while (!costmap_ros_->isCurrent()) {
        r.sleep();
      }

      updateGlobalPath();

      computeAndPublishVelocity();

      if (isGoalReached()) {
        RCLCPP_INFO(get_logger(), "Reached the goal!");
        break;
      }

      if (!loop_rate.sleep()) {
        RCLCPP_WARN(get_logger(),
                    "Control loop missed its desired rate of %.4fHz",
                    controller_frequency_);
      }
    }
  } catch (nav2_core::PlannerException &e) {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    publishZeroVelocity();
    action_server_->terminate_current();
    return;
  } catch (std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    publishZeroVelocity();
    std::shared_ptr<Action::Result> result = std::make_shared<Action::Result>();
    action_server_->terminate_current(result);
    return;
  }

  RCLCPP_DEBUG(get_logger(), "Controller succeeded, setting result");

  publishZeroVelocity();

  // TODO(orduno) #861 Handle a pending preemption and set controller name
  action_server_->succeeded_current();
}

void ControllerServer::setPlannerPath(
    const vrobot_local_planner::msg::Path &path) {
  RCLCPP_DEBUG(get_logger(), "Providing path to the controller %s",
               current_controller_.c_str());
  if (path.poses.empty()) {
    throw std::runtime_error("Invalid path, Path is empty.");
  }
  controllers_[current_controller_]->setPlan(path);

  end_pose_                 = path.poses.back();
  end_pose_.header.frame_id = path.header.frame_id;
  goal_checkers_[current_goal_checker_]->reset();

  RCLCPP_DEBUG(get_logger(), "Path end point is (%.2f, %.2f)",
               end_pose_.pose.position.x, end_pose_.pose.position.y);

  current_path_ = path;
}

void ControllerServer::computeAndPublishVelocity() {
  vrobot_local_planner::msg::PlannerPose pose;

  if (!getRobotPose(pose)) {
    throw std::runtime_error("Failed to obtain robot pose");
  }

  geometry_msgs::msg::Twist twist = getThresholdedTwist(odom_sub_->getTwist());

  geometry_msgs::msg::TwistStamped cmd_vel_2d;

  try {
    cmd_vel_2d = controllers_[current_controller_]->computeVelocityCommands(
        pose, twist, goal_checkers_[current_goal_checker_].get());
  } catch (nav2_core::PlannerException &e) {
    if (failure_tolerance_ > 0 || failure_tolerance_ == -1.0) {
      RCLCPP_WARN(this->get_logger(), "%s", e.what());
      cmd_vel_2d.twist.angular.x = 0;
      cmd_vel_2d.twist.angular.y = 0;
      cmd_vel_2d.twist.angular.z = 0;
      cmd_vel_2d.twist.linear.x  = 0;
      cmd_vel_2d.twist.linear.y  = 0;
      cmd_vel_2d.twist.linear.z  = 0;
      cmd_vel_2d.header.frame_id = costmap_ros_->getBaseFrameID();
      cmd_vel_2d.header.stamp    = now();
    } else {
      throw nav2_core::PlannerException(e.what());
    }
  }

  std::shared_ptr<Action::Feedback> feedback =
      std::make_shared<Action::Feedback>();
  feedback->speed =
      std::hypot(cmd_vel_2d.twist.linear.x, cmd_vel_2d.twist.linear.y);

  // Find the closest pose to current pose on global path
  vrobot_local_planner::msg::Path &current_path = current_path_;
  auto find_closest_pose_idx                    = [&pose, &current_path]() {
    size_t closest_pose_idx = 0;
    double curr_min_dist    = std::numeric_limits<double>::max();
    for (size_t curr_idx = 0; curr_idx < current_path.poses.size();
         ++curr_idx) {
      double curr_dist =
          vrobot_local_planner::geometry_utils::euclidean_distance(
                                 pose, current_path.poses[curr_idx]);
      if (curr_dist < curr_min_dist) {
        curr_min_dist    = curr_dist;
        closest_pose_idx = curr_idx;
      }
    }
    return closest_pose_idx;
  };

  feedback->distance_to_goal =
      vrobot_local_planner::geometry_utils::calculate_path_length(
          current_path_, find_closest_pose_idx());
  action_server_->publish_feedback(feedback);

  RCLCPP_DEBUG(get_logger(), "Publishing velocity at time %.2f",
               now().seconds());
  publishVelocity(cmd_vel_2d);
}

void ControllerServer::updateGlobalPath() {
  if (action_server_->is_preempt_requested()) {
    RCLCPP_INFO(get_logger(), "Passing new path to controller.");
    auto        goal = action_server_->accept_pending_goal();
    std::string current_controller;
    if (findControllerId(goal->controller_id, current_controller)) {
      current_controller_ = current_controller;
    } else {
      RCLCPP_INFO(get_logger(),
                  "Terminating action, invalid controller %s requested.",
                  goal->controller_id.c_str());
      action_server_->terminate_current();
      return;
    }
    std::string current_goal_checker;
    if (findGoalCheckerId(goal->goal_checker_id, current_goal_checker)) {
      current_goal_checker_ = current_goal_checker;
    } else {
      RCLCPP_INFO(get_logger(),
                  "Terminating action, invalid goal checker %s requested.",
                  goal->goal_checker_id.c_str());
      action_server_->terminate_current();
      return;
    }
    setPlannerPath(goal->path);
  }
}

void ControllerServer::publishVelocity(
    const geometry_msgs::msg::TwistStamped &velocity) {
  auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>(velocity.twist);
  if (vel_publisher_->get_subscription_count() > 0) {
    vel_publisher_->publish(std::move(cmd_vel));
  }
}

void ControllerServer::publishZeroVelocity() {
  geometry_msgs::msg::TwistStamped velocity;
  velocity.twist.angular.x = 0;
  velocity.twist.angular.y = 0;
  velocity.twist.angular.z = 0;
  velocity.twist.linear.x  = 0;
  velocity.twist.linear.y  = 0;
  velocity.twist.linear.z  = 0;
  velocity.header.frame_id = costmap_ros_->getBaseFrameID();
  velocity.header.stamp    = now();
  publishVelocity(velocity);
}

bool ControllerServer::isGoalReached() {
  vrobot_local_planner::msg::PlannerPose pose;

  if (!getRobotPose(pose)) {
    return false;
  }

  geometry_msgs::msg::Twist twist = getThresholdedTwist(odom_sub_->getTwist());

  geometry_msgs::msg::PoseStamped transformed_end_pose;
  rclcpp::Duration                tolerance(
                     rclcpp::Duration::from_seconds(costmap_ros_->getTransformTolerance()));

  geometry_msgs::msg::PoseStamped end_pose_stamped;
  end_pose_stamped.pose   = end_pose_.pose;
  end_pose_stamped.header = end_pose_.header;

  nav_2d_utils::transformPose(
      costmap_ros_->getTfBuffer(), costmap_ros_->getGlobalFrameID(),
      end_pose_stamped, transformed_end_pose, tolerance);

  return goal_checkers_[current_goal_checker_]->isGoalReached(
      pose.pose, transformed_end_pose.pose, twist);
}

bool ControllerServer::getRobotPose(
    vrobot_local_planner::msg::PlannerPose &pose) {
  // vrobot_local_planner::msg::PlannerPose current_pose;
  geometry_msgs::msg::PoseStamped current_pose;

  if (!costmap_ros_->getRobotPose(current_pose)) {
    return false;
  }
  pose.pose   = current_pose.pose;
  pose.header = current_pose.header;
  return true;
}

} // namespace vrobot_local_planner

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(vrobot_local_planner::ControllerServer)
