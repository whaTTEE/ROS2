#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class TurtlebotControl : public rclcpp::Node
{
public:
  TurtlebotControl() : Node("turtlebot_control")
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&TurtlebotControl::move_turtlebot, this));
  }

private:
  void move_turtlebot()
  {
    auto message = geometry_msgs::msg::Twist();
    message.linear.x = 0.5;  // Move forward with a linear velocity of 0.5 m/s
    message.angular.z = 0.0; // No angular velocity (no rotation)
    publisher_->publish(message);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtlebotControl>());
  rclcpp::shutdown();
  return 0;
}
