cmake_minimum_required(VERSION 3.8)
project(vrobot_local_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(THIS_PACKAGE_DEPENDENCIES 
  rclcpp
  rclcpp_components
  nav2_msgs
  nav_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  nav_2d_msgs
  nav2_costmap_2d
  nav_2d_utils
  pluginlib
)

foreach(DEPENDENCY ${THIS_PACKAGE_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()



# =======================================Interface definition=======================================
file(GLOB ${PROJECT_NAME}_interface_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} msg/*)
file(GLOB ${PROJECT_NAME}_srv_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} srv/*)
file(GLOB ${PROJECT_NAME}_action_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} action/*)
rosidl_generate_interfaces(
   ${PROJECT_NAME}
   ${${PROJECT_NAME}_interface_files}
   ${${PROJECT_NAME}_srv_files}
   ${${PROJECT_NAME}_action_files}
   DEPENDENCIES ${THIS_PACKAGE_DEPENDENCIES}
)
ament_export_dependencies(rosidl_default_runtime)
# ===================================End of Interface definition===================================


include_directories(
  include
)

# LIBRARY
file(GLOB COMMON_SRC src/common/*.cpp)
file(GLOB UTILS_SRC src/utils/*.cpp)
add_library(${PROJECT_NAME}_lib SHARED
  ${COMMON_SRC}
  ${UTILS_SRC}
  src/controller_server.cpp
)
target_include_directories(${PROJECT_NAME}_lib  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_lib
  ${THIS_PACKAGE_DEPENDENCIES}
)
rosidl_target_interfaces(${PROJECT_NAME}_lib 
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

# PLUGINS
add_library(simple_goal_checker SHARED src/plugins/simple_goal_checker.cpp)
ament_target_dependencies(simple_goal_checker ${THIS_PACKAGE_DEPENDENCIES})

add_library(stopped_goal_checker SHARED src/plugins/stopped_goal_checker.cpp)
ament_target_dependencies(stopped_goal_checker ${THIS_PACKAGE_DEPENDENCIES})

add_library(rpp_controller SHARED src/plugins/rpp_controller.cpp)
ament_target_dependencies(rpp_controller ${THIS_PACKAGE_DEPENDENCIES})
rosidl_target_interfaces(rpp_controller 
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)


add_executable(${PROJECT_NAME}_main
    src/main.cpp
)
target_link_libraries(${PROJECT_NAME}_main 
  ${PROJECT_NAME}_lib
)

install(TARGETS
  ${PROJECT_NAME}_lib simple_goal_checker stopped_goal_checker rpp_controller
  EXPORT ${PROJECT_NAME}_lib_export
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  ${PROJECT_NAME}_main
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(
  simple_goal_checker
  stopped_goal_checker
  rpp_controller
)
ament_export_dependencies(${THIS_PACKAGE_DEPENDENCIES})
ament_export_targets(${PROJECT_NAME}_lib_export HAS_LIBRARY_TARGET)
pluginlib_export_plugin_description_file(vrobot_local_planner plugins.xml)

ament_package()
