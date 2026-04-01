#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "irobot_create_msgs/msg/interface_buttons.hpp"
#include "irobot_create_msgs/msg/lightring_leds.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

// Movement parameters
static constexpr double LINEAR_VELOCITY = 2.0; // m/s
static constexpr double DISTANCE = 5.0;        // m
// Duration to move specified distance
static constexpr double MOVE_DURATION = DISTANCE / LINEAR_VELOCITY; // s

class TurtleBot4FirstNode : public rclcpp::Node
{
public:
  TurtleBot4FirstNode() : Node("turtlebot4_first_cpp_node"), moving_(false)
  {
    // Subscribe to the /interface_buttons topic
    interface_buttons_subscriber_ =
        this->create_subscription<irobot_create_msgs::msg::InterfaceButtons>(
            "/interface_buttons",
            rclcpp::SensorDataQoS(),
            std::bind(&TurtleBot4FirstNode::interface_buttons_callback,
                      this,
                      std::placeholders::_1));

    // Publisher for cmd_vel to move the robot
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    RCLCPP_INFO(this->get_logger(),
      "Node Ready. Press Button 1 on the robot to move forward %.2f meters.",
      DISTANCE);
  }

private:
  // Interface buttons subscription callback
  void interface_buttons_callback(
      const irobot_create_msgs::msg::InterfaceButtons::SharedPtr create3_buttons_msg)
  {
    if (create3_buttons_msg->button_1.is_pressed)
    {
      RCLCPP_INFO(this->get_logger(), "Button 1: Moving forward...");
      move_forward();
    }

    if (create3_buttons_msg->button_2.is_pressed)
    {
      RCLCPP_INFO(this->get_logger(), "Button 2: Stopping the robot");
      stop_robot();
    }
  }

  void move_forward()
  {
    moving_ = true;

    auto twist = geometry_msgs::msg::Twist();
    twist.linear.x = LINEAR_VELOCITY;
    twist.angular.z = 0.0;
    cmd_vel_publisher_->publish(twist);

    move_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(MOVE_DURATION),
      [this]() {
          RCLCPP_INFO(this->get_logger(), "Reached target distance. Stopping.");
          stop_robot();
      });
  }

  void stop_robot()
  {
    moving_ = false;

    if (move_timer_) {
      move_timer_->cancel();
      move_timer_.reset();
    }

    auto twist = geometry_msgs::msg::Twist();
    cmd_vel_publisher_->publish(twist);
  }

  bool moving_;

  rclcpp::Subscription<irobot_create_msgs::msg::InterfaceButtons>::SharedPtr interface_buttons_subscriber_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

  rclcpp::TimerBase::SharedPtr move_timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtleBot4FirstNode>());
  rclcpp::shutdown();

  return 0;
}
