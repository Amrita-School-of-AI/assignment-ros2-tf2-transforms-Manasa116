#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

using namespace std::chrono_literals;

class SimpleTFEcho : public rclcpp::Node
{
public:
  SimpleTFEcho()
  : Node("simple_tf_echo"),
    buffer_(this->get_clock()),
    listener_(buffer_)
  {
    timer_ = this->create_wall_timer(
      1s, std::bind(&SimpleTFEcho::timer_callback, this));
  }

private:
  void timer_callback()
  {
    try
    {
      auto transform = buffer_.lookupTransform(
        "world", "robot", tf2::TimePointZero);

      RCLCPP_INFO(this->get_logger(),
        "At time %.2f\n- Translation: [%.3f, %.3f, %.3f]\n- Rotation: in Quaternion [%.3f, %.3f, %.3f, %.3f]\n",
        transform.header.stamp.sec + transform.header.stamp.nanosec * 1e-9,
        transform.transform.translation.x,
        transform.transform.translation.y,
        transform.transform.translation.z,
        transform.transform.rotation.x,
        transform.transform.rotation.y,
        transform.transform.rotation.z,
        transform.transform.rotation.w);
    }
    catch (tf2::TransformException & ex)
    {
      RCLCPP_WARN(this->get_logger(), "%s", ex.what());
    }
  }

  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleTFEcho>());
  rclcpp::shutdown();
  return 0;
}
