cmake_minimum_required(VERSION 3.8)
project(ros2_tf2)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(simple_tf_echo src/simple_tf_echo.cpp)

ament_target_dependencies(simple_tf_echo
  rclcpp
  tf2_ros
  geometry_msgs
)

install(TARGETS
  simple_tf_echo
  DESTINATION lib/${PROJECT_NAME}
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
