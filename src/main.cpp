#include "vrobot_local_planner/controller_server.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;

  auto node = std::make_shared<vrobot_local_planner::ControllerServer>();
  try {
    node->on_configure();
  } catch (const std::exception &e) {
    RCLCPP_ERROR(node->get_logger(),
                 "Failed to configure controller server: %s", e.what());
    return 1;
  } catch (...) {
    RCLCPP_ERROR(node->get_logger(), "Failed to configure controller server");
    return 1;
  }
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}