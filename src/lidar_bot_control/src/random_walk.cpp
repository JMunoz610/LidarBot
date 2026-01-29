#include <chrono>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std::chrono_literals;

class RandomWalk : public rclcpp::Node
{
public:
  RandomWalk()
  : Node("random_walk"), count_(0)
  {
    // publisher callback
    auto vel_callback =
      [this]() -> void {
        
        geometry_msgs::msg::Twist msg;
        msg.linear.x = 0.5;
        msg.angular.z = 0.0;

        RCLCPP_INFO(this->get_logger(), "Publishing cmd_vel");
        this->publisher_->publish(msg);
      };

    // subscriber callback
    auto pose_callback =
      [this](nav_msgs::msg::Odometry::UniquePtr msg) -> void {
        const auto &position = msg->pose.pose.position;
        // const auto &linear = msg->twist.twist.linear;
        // const auto &angular = msg->twist.twist.angular;
        const auto &orientation = msg->pose.pose.orientation;

        tf2::Quaternion q(orientation.x, orientation.y, orientation.z, orientation.w);

        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        // send to global variable
        pose_.x = position.x;
        pose_.y = position.y;
        pose_.heading = yaw;
        RCLCPP_INFO(this->get_logger(), "Pose: x=%.2f y=%.2f heading=%.2f", pose_.x, pose_.y, pose_.heading*180.0/M_PI);
      };

    // publisher
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    vel_timer_ = this->create_wall_timer(500ms, vel_callback);

    // subscriber
    subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, pose_callback);
  }

private:
  struct Pose2D{
    double x = 0.0;
    double y = 0.0;
    double heading = 0.0;
  };
  Pose2D pose_;
  rclcpp::TimerBase::SharedPtr vel_timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RandomWalk>());
  rclcpp::shutdown();
  return 0;
}