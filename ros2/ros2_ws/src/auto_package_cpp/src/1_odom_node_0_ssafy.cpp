/*
 * Odometry Node
 * 
 * 기능:
 * - 로봇의 위치와 방향 정보를 추적하고 발행
 * - TF 트리에서 base_link와 laser 프레임 간의 변환 관계 유지
 * 
 * 토픽:
 * - 구독:
 *   - /turtlebot_status: 로봇의 상태 정보
 * - 발행:
 *   - /odom: 로봇의 위치와 방향 정보
 * - TF 발행:
 *   - map -> base_link
 *   - base_link -> laser
 */

#include "rclcpp/rclcpp.hpp"
#include "ssafy_msgs/msg/turtlebot_status.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "builtin_interfaces/msg/time.hpp"
#include <cmath>
#include <cstdint>
#include <iso646.h> // Alternative operator spellings

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Quaternion from Euler angles helper function
// Replicating squaternion's functionality
struct Quaternion {
  double x, y, z, w;
  
  static Quaternion from_euler(double roll, double pitch, double yaw) {
    Quaternion q;
    // Convert Euler angles to quaternion
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;
    
    return q;
  }
};

class OdomNode : public rclcpp::Node {
public:
  OdomNode() : Node("odom"), is_status_(false), is_calc_theta_(false),
               x_(0.0), y_(0.0), theta_(0.0), x1_(0.0), y1_(0.0), theta1_(0.0) {
    
    // Create subscription
    status_sub_ = this->create_subscription<ssafy_msgs::msg::TurtlebotStatus>(
      "/turtlebot_status", 10,
      std::bind(&OdomNode::status_callback, this, std::placeholders::_1));
    
    // Create publisher
    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    
    // Initialize TF broadcaster
    broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    
    // Initialize odometry message
    odom_msg_.header.frame_id = "map";
    odom_msg_.child_frame_id = "base_link";
    
    // Initialize base_link transform
    base_link_transform_.header.frame_id = "map";
    base_link_transform_.child_frame_id = "base_link";
    
    // Initialize laser transform
    laser_transform_.header.frame_id = "base_link";
    laser_transform_.child_frame_id = "laser";
    laser_transform_.transform.translation.x = 0.0;
    laser_transform_.transform.translation.y = 0.0;
    laser_transform_.transform.translation.z = 1.0;
    laser_transform_.transform.rotation.w = 1.0;
  }

private:
  void status_callback(const ssafy_msgs::msg::TurtlebotStatus::SharedPtr msg) {
    if (not is_status_) {
      is_status_ = true;
      prev_time_ = this->now();
      x1_ = msg->twist.angular.x;  // 초기 x 좌표 저장
      y1_ = msg->twist.angular.y;  // 초기 y 좌표 저장
      theta1_ = msg->twist.linear.z;  // 초기 방향 저장
    } else {
      rclcpp::Time current_time = this->now();
      double period = (current_time - prev_time_).nanoseconds() / 1.0e9; // Convert to seconds
      
      double linear_x = msg->twist.linear.x;
      double angular_z = msg->twist.angular.z;
      
      // 기존 적분 방식 대신 로봇이 제공하는 절대 좌표와 방향 정보 사용
      x_ = msg->twist.angular.x - x1_;  // 로봇의 절대 x 좌표
      y_ = msg->twist.angular.y - y1_;  // 로봇의 절대 y 좌표
      // 상대적인 방향 계산 (라디안 단위)
      theta_ = msg->twist.linear.z * (M_PI / 180.0);
      
      // 각도가 -pi에서 pi 범위를 벗어나지 않도록 정규화
      if (theta_ > M_PI) {
        theta_ -= 2.0 * M_PI;
      } else if (theta_ < -M_PI) {
        theta_ += 2.0 * M_PI;
      }
      
      Quaternion q = Quaternion::from_euler(0.0, 0.0, theta_);
      
      // Get current ROS time for message headers
      builtin_interfaces::msg::Time current_ros_time;
      current_ros_time.sec = static_cast<int32_t>(current_time.seconds());
      current_ros_time.nanosec = static_cast<uint32_t>(current_time.nanoseconds() % 1000000000);
      
      // Update transform stamps
      base_link_transform_.header.stamp = current_ros_time;
      laser_transform_.header.stamp = current_ros_time;
      
      // Update base_link transform
      base_link_transform_.transform.translation.x = x_;
      base_link_transform_.transform.translation.y = y_;
      base_link_transform_.transform.rotation.x = q.x;
      base_link_transform_.transform.rotation.y = q.y;
      base_link_transform_.transform.rotation.z = q.z;
      base_link_transform_.transform.rotation.w = q.w;
      
      // Update odometry message
      odom_msg_.header.stamp = current_ros_time;
      odom_msg_.pose.pose.position.x = x_;
      odom_msg_.pose.pose.position.y = y_;
      odom_msg_.pose.pose.orientation.x = q.x;
      odom_msg_.pose.pose.orientation.y = q.y;
      odom_msg_.pose.pose.orientation.z = q.z;
      odom_msg_.pose.pose.orientation.w = q.w;
      odom_msg_.twist.twist.linear.x = linear_x;
      odom_msg_.twist.twist.angular.z = angular_z;
      
      // Broadcast transforms
      std::vector<geometry_msgs::msg::TransformStamped> transforms;
      transforms.push_back(base_link_transform_);
      transforms.push_back(laser_transform_);
      broadcaster_->sendTransform(transforms);
      
      // Publish odometry
      odom_publisher_->publish(odom_msg_);
      
      // Update previous time
      prev_time_ = current_time;
    }
  }

  // Subscriber
  rclcpp::Subscription<ssafy_msgs::msg::TurtlebotStatus>::SharedPtr status_sub_;
  
  // Publisher
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  
  // TF Broadcaster
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
  
  // Messages
  nav_msgs::msg::Odometry odom_msg_;
  geometry_msgs::msg::TransformStamped base_link_transform_;
  geometry_msgs::msg::TransformStamped laser_transform_;
  
  // State variables
  bool is_status_;
  bool is_calc_theta_;
  double x_;
  double y_;
  double theta_;
  double x1_;  // 초기 x 좌표 오프셋
  double y1_;  // 초기 y 좌표 오프셋
  double theta1_;  // 초기 방향 저장
  rclcpp::Time prev_time_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OdomNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}