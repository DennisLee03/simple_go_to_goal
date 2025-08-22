cmake_minimum_required(VERSION 3.8)
project(demo_diff_drive_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# generate custom interface
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/SimpleGoToGoal.action"
  DEPENDENCIES
  action_msgs
  ADD_LINTER_TESTS
)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")


include_directories(include)

add_executable(simple_go_to_goal src/simple_go_to_goal.cpp src/PID_control.cpp)
ament_target_dependencies(simple_go_to_goal rclcpp nav_msgs geometry_msgs tf2 tf2_geometry_msgs)

add_executable(simple_go_to_goal_action_server 
  src/simple_go_to_goal_action_server.cpp src/PID_control.cpp
)
ament_target_dependencies(simple_go_to_goal_action_server 
  rclcpp 
  nav_msgs 
  geometry_msgs 
  tf2 
  tf2_geometry_msgs
)
target_link_libraries(simple_go_to_goal_action_server "${cpp_typesupport_target}")



install(TARGETS
  simple_go_to_goal
  simple_go_to_goal_action_server
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch description config worlds
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
