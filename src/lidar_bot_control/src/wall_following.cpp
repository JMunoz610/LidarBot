#include <chrono>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std::chrono_literals;

class WallFollowing : public rclcpp::Node
{
public:
  WallFollowing()
  : Node("wall_following"), count_(0)
  {
    // publisher callback
    auto control_callback =
      [this]() -> void {
        if(laser_scan_.ranges.empty()){return;}

        // control logic
        float min_value = std::numeric_limits<float>::infinity();
        
        int min_index = (-desired_angle_range-laser_scan_.angle_min)/laser_scan_.angle_increment;
        int max_index = (desired_angle_range-laser_scan_.angle_min)/laser_scan_.angle_increment;
        
        for(int i = min_index; i <= max_index; i++){
          float scan_value = laser_scan_.ranges[i];
          if(std::isfinite(scan_value) && scan_value < min_value){
            min_value = scan_value;
          }
        }
        RCLCPP_INFO(this->get_logger(), "Minimum distance to wall is: %f", min_value);

        /*
        Going to add logic to control both heading and distance error
        Will have the heading be based on a specified lidar point that will be depended on which side the wall is on (left or right)
        Use average distance for distance to wall based on a small sector of lidar points (-80 to - 100)
        Then make github page for it and post it
        */
        
  
        // send message
        geometry_msgs::msg::Twist msg;
        
        RCLCPP_INFO(this->get_logger(), "Angular Vel: %f", ang_vel);
        this->publisher_->publish(msg);
      };

    // subscriber callbacks
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
        //RCLCPP_INFO(this->get_logger(), "Pose: x=%.2f y=%.2f heading=%.2f", pose_.x, pose_.y, pose_.heading*180.0/M_PI);
      };

    auto scan_callback =
      [this](sensor_msgs::msg::LaserScan::UniquePtr msg) -> void {
        laser_scan_ = *msg;
        //RCLCPP_INFO(this->get_logger(), "Number of scan values %zu", msg->ranges.size());
      };

    // publisher
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    vel_timer_ = this->create_wall_timer(50ms, control_callback);

    // subscribers
    pose_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 50, pose_callback);
    scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, scan_callback);
  }

private:
  rclcpp::TimerBase::SharedPtr vel_timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
  sensor_msgs::msg::LaserScan laser_scan_;
  size_t count_;
  
  // custom stuff
  struct Pose2D{
    double x = 0.0;
    double y = 0.0;
    double heading = 0.0;
  };
  Pose2D pose_;
  float min_distance_to_wall = 1.0; //meters
  float desired_angle_range = 25*M_PI/180.0;

  // For PID Control

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WallFollowing>());
  rclcpp::shutdown();
  return 0;
}