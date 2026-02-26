#include <memory>
#include <chrono>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

using namespace std::chrono_literals;

class TFBroadcaster : public rclcpp::Node
{
public:
  TFBroadcaster()
  : Node("tf_broadcaster")
  {
    broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Store initial time so motion starts at t = 0
    start_time_ = this->get_clock()->now();

    timer_ = this->create_wall_timer(
      100ms, std::bind(&TFBroadcaster::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto now = this->get_clock()->now();

    // Time since node started
    double t = (now - start_time_).seconds();

    // Circular motion (radius = 2)
    double x = 2.0 * std::cos(t);
    double y = 2.0 * std::sin(t);

    geometry_msgs::msg::TransformStamped transform;

    transform.header.stamp = now;
    transform.header.frame_id = "world";
    transform.child_frame_id = "robot";

    // Translation
    transform.transform.translation.x = x;
    transform.transform.translation.y = y;
    transform.transform.translation.z = 0.0;

    // Identity quaternion (no rotation)
    transform.transform.rotation.x = 0.0;
    transform.transform.rotation.y = 0.0;
    transform.transform.rotation.z = 0.0;
    transform.transform.rotation.w = 1.0;

    broadcaster_->sendTransform(transform);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  rclcpp::Time start_time_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TFBroadcaster>());
  rclcpp::shutdown();
  return 0;
}
