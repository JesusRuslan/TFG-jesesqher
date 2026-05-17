#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "irobot_create_msgs/msg/interface_buttons.hpp"
#include "irobot_create_msgs/msg/lightring_leds.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;

// Movement parameters
static constexpr double LINEAR_VELOCITY = 0.3; // m/s - close to real limit
static constexpr double DISTANCE = 0.5;        // m

class TurtleBot4FirstNode : public rclcpp::Node
{
public:
  // == Constructor ============================================================================================
  TurtleBot4FirstNode()
      : Node("turtlebot4_first_cpp_node"),
        moving_(false),
        odom_received_(false),
        start_x_(0.0), start_y_(0.0)
  {
    // == Subscription to interface buttons to detect button presses ==========================================
    interface_buttons_subscriber_ =
        this->create_subscription<irobot_create_msgs::msg::InterfaceButtons>(
            "/interface_buttons",
            rclcpp::SensorDataQoS(),
            std::bind(&TurtleBot4FirstNode::interface_buttons_callback,
                      this,
                      std::placeholders::_1));

    // == Odometry subscription to track robot's position =========================================================
    odom_subscriber_ =
        this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom",
            rclcpp::SensorDataQoS(),
            std::bind(&TurtleBot4FirstNode::odometry_callback,
                      this,
                      std::placeholders::_1));

    // == Publisher for cmd_vel to control the robot's movement =======================================================
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    RCLCPP_INFO(this->get_logger(),
                "Node Ready. Press Button 1 on the robot to move forward %.2f meters.",
                DISTANCE);
  }

private:
  // == Odometry callback ============================================================================================
  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;

    // Log current position for debugging every 1 second
    auto now = this->get_clock()->now();
    if ((now - last_log_time_).seconds() >= 1.0)
    {
      RCLCPP_INFO(this->get_logger(),
                  "Current State: (x, y) = (%.2f, %.2f); linear_vel = %.2f",
                  current_x_, current_y_,
                  msg->twist.twist.linear.x);
      last_log_time_ = now;
    }

    if (!odom_received_)
    {
      odom_received_ = true;
    }

    // == Check distance traveled ============================================================================
    if (moving_)
    {
      double dx = current_x_ - start_x_;
      double dy = current_y_ - start_y_;
      double distance_traveled = std::sqrt(dx * dx + dy * dy);

      if (distance_traveled >= DISTANCE)
      {
        RCLCPP_INFO(this->get_logger(),
                    "Target distance reached: %.2f meters. Stopping the robot.",
                    distance_traveled);
        stop_robot();
      }
    }
  }

  // == Interface buttons subscription callback ================================================================
  void interface_buttons_callback(
      const irobot_create_msgs::msg::InterfaceButtons::SharedPtr create3_buttons_msg)
  {
    if (create3_buttons_msg->button_1.is_pressed)
    {
      if (!odom_received_)
      {
        RCLCPP_WARN(this->get_logger(),
                    "Odometry data not received yet. Cannot move.");
        return;
      }
      RCLCPP_INFO(this->get_logger(),
                  "Button 1: Moving forward %.2f meters from (%.2f, %.2f)...",
                  DISTANCE,
                  start_x_, start_y_);
      move_forward();
    }

    if (create3_buttons_msg->button_2.is_pressed)
    {
      RCLCPP_INFO(this->get_logger(),
                  "Button 2: Stopping the robot");
      stop_robot();
    }
  }

  // ==== Movement control methods ==================================================================================

  void move_forward()
  {
    start_x_ = current_x_;
    start_y_ = current_y_;
    moving_ = true;

    cmd_vel_timer_ = this->create_wall_timer(
        100ms,
        [this]()
        {
          if (!moving_)
          {
            cmd_vel_timer_->cancel();
            return;
          }
          auto twist = geometry_msgs::msg::Twist();
          twist.linear.x = LINEAR_VELOCITY;
          twist.angular.z = 0.0;
          cmd_vel_publisher_->publish(twist);
        });
  }

  void stop_robot()
  {
    moving_ = false;

    if (cmd_vel_timer_)
    {
      cmd_vel_timer_->cancel();
      cmd_vel_timer_.reset();
    }

    auto twist = geometry_msgs::msg::Twist();
    cmd_vel_publisher_->publish(twist);
  }

  // ==== Member variables ==================================================================================

  bool moving_;
  bool odom_received_;

  double start_x_, start_y_;
  double current_x_{0.0}, current_y_{0.0};

  rclcpp::Time last_log_time_{0, 0, RCL_ROS_TIME};
  rclcpp::TimerBase::SharedPtr cmd_vel_timer_;

  rclcpp::Subscription<irobot_create_msgs::msg::InterfaceButtons>::SharedPtr interface_buttons_subscriber_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtleBot4FirstNode>());
  rclcpp::shutdown();

  return 0;
}
