[![Review Assignment Due Date](https://classroom.github.com/assets/deadline-readme-button-22041afd0340ce965d47ae6ef1cefeee28c7c493a6346c4f15d667ab976d596c.svg)](https://classroom.github.com/a/1bGIVTPs)
# Assignment: ROS2 TF2 Transforms

**Course:** ROS2 Advanced  
**Build System:** ament_cmake

## Objective

The goal of this assignment is to verify your understanding of:

1. How to broadcast transforms using TF2.
2. How to listen to transforms.
3. Understanding coordinate frames in robotics.

## Problem Statement

You must complete the provided skeleton code to create a transform broadcaster that publishes the transform between a `world` frame and a `robot` frame.

### Requirements

1. **Source Code (`src/tf_broadcaster.cpp`)**:
   - Implement a class `TFBroadcaster` that inherits from `rclcpp::Node`.
   - Initialize the node with the name `"tf_broadcaster"`.
   - Create a `TransformBroadcaster`.
   - Create a timer that fires every **100ms**.
   - In the timer callback:
     - Create and broadcast a transform from `"world"` to `"robot"` frame.
     - The robot should move in a circle:
       - x = 2.0 \* cos(time)
       - y = 2.0 \* sin(time)
       - z = 0.0
     - Set rotation (quaternion) to identity (no rotation).

2. **Build Configuration (`CMakeLists.txt`)**:
   - Add executable and link `tf2_ros` and `geometry_msgs`.

3. **Package Metadata (`package.xml`)**:
   - Add required dependencies.

## How to Test Locally

```bash
colcon build --packages-select ros2_tf2
source install/setup.bash
ros2 run ros2_tf2 tf_broadcaster

# In another terminal, view the transform:
ros2 run tf2_ros tf2_echo world robot

# Or use rviz2:
rviz2
# Add TF display, set Fixed Frame to "world"
```

#### Expected Output from `tf2_echo`:

```shell
At time X.X
- Translation: [2.000, 0.000, 0.000]
- Rotation: in Quaternion [0.000, 0.000, 0.000, 1.000]
At time X.X
- Translation: [1.999, 0.063, 0.000]
...
```
