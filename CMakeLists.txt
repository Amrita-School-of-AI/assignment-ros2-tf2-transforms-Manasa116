cmake_minimum_required(VERSION 3.8)
project(ros2_tf2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

# tf_broadcaster
add_executable(tf_broadcaster src/tf_broadcaster.cpp)
ament_target_dependencies(tf_broadcaster
  rclcpp
  tf2_ros
  geometry_msgs
)

# simple_tf_echo
add_executable(simple_tf_echo src/simple_tf_echo.cpp)
ament_target_dependencies(simple_tf_echo
  rclcpp
  tf2_ros
  geometry_msgs
)

# Install BOTH executables together
install(TARGETS
  tf_broadcaster
  simple_tf_echo
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
