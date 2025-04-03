#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"

class NewOdomNode : public rclcpp::Node {
public:
  NewOdomNode() : Node("new_odom"), first_msg_(true), cmd_vel_received_(false) {
    // Pose 토픽 구독
    pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
      "/robots/fest_1/position", 10,
      std::bind(&NewOdomNode::pose_callback, this, std::placeholders::_1));
    
    // cmd_vel 토픽 구독 추가
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10,
      std::bind(&NewOdomNode::cmd_vel_callback, this, std::placeholders::_1));
    
    // Odom 발행자 생성
    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    
    // StaticTransformBroadcaster 대신 TransformBroadcaster 사용
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    
    // Odometry 메시지 초기화
    odom_msg_.header.frame_id = "map";
    odom_msg_.child_frame_id = "base_link";
    
    // Transform 초기화
    transform_.header.frame_id = "map";
    transform_.child_frame_id = "base_link";
    
    // 레이저 transform 초기화
    laser_transform_.header.frame_id = "base_link";
    laser_transform_.child_frame_id = "laser";
    laser_transform_.transform.translation.x = 0.0;
    laser_transform_.transform.translation.y = 0.0;
    laser_transform_.transform.translation.z = 1.0;
    laser_transform_.transform.rotation.w = 1.0;

    // cmd_vel 초기화 - linear.x와 angular.z만 사용
    current_cmd_vel_.linear.x = 0.0;
    current_cmd_vel_.angular.z = 0.0;

    // 나머지 twist 값들 0으로 초기화
    odom_msg_.twist.twist.linear.y = 0.0;
    odom_msg_.twist.twist.linear.z = 0.0;
    odom_msg_.twist.twist.angular.x = 0.0;
    odom_msg_.twist.twist.angular.y = 0.0;

    // laser transform은 한 번만 발행
    static_broadcaster_->sendTransform(laser_transform_);
  }

private:
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    // linear.x와 angular.z만 저장
    current_cmd_vel_.linear.x = msg->linear.x;
    current_cmd_vel_.angular.z = msg->angular.z;
    cmd_vel_received_ = true;
    last_cmd_vel_time_ = this->now();
  }

  void pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg) {
    auto current_time = this->now();
    
    if (first_msg_) {
      // 초기 위치 저장
      initial_pose_ = *msg;
      first_msg_ = false;
    }
    
    // Odometry 메시지 업데이트 - 상대 위치 계산
    odom_msg_.header.stamp = current_time;
    odom_msg_.pose.pose.position.x = msg->position.x - initial_pose_.position.x;
    odom_msg_.pose.pose.position.y = msg->position.y - initial_pose_.position.y;
    odom_msg_.pose.pose.position.z = 0.0;  // z는 0으로 고정
    odom_msg_.pose.pose.orientation = msg->orientation;
    
    // cmd_vel 타임아웃 체크 (예: 0.5초)
    if (!cmd_vel_received_ || 
        (current_time - last_cmd_vel_time_).seconds() > 0.5) {
      // 타임아웃 발생 시 속도를 0으로 설정
      current_cmd_vel_.linear.x = 0.0;
      current_cmd_vel_.angular.z = 0.0;
    }
    
    // twist 정보 업데이트 - linear.x와 angular.z만 설정
    odom_msg_.twist.twist.linear.x = current_cmd_vel_.linear.x;
    odom_msg_.twist.twist.angular.z = current_cmd_vel_.angular.z;
    
    // Transform 업데이트 - 상대 위치 사용
    transform_.header.stamp = current_time;
    transform_.transform.translation.x = odom_msg_.pose.pose.position.x;
    transform_.transform.translation.y = odom_msg_.pose.pose.position.y;
    transform_.transform.translation.z = 0.0;
    transform_.transform.rotation = msg->orientation;
    
    // Transform 브로드캐스트 - base_link transform만 동적으로 발행
    tf_broadcaster_->sendTransform(transform_);
    
    // Odometry 발행
    odom_publisher_->publish(odom_msg_);
  }

  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
  nav_msgs::msg::Odometry odom_msg_;
  geometry_msgs::msg::TransformStamped transform_;
  geometry_msgs::msg::TransformStamped laser_transform_;
  geometry_msgs::msg::Twist current_cmd_vel_;
  geometry_msgs::msg::Pose initial_pose_;  // 초기 위치 저장용
  bool cmd_vel_received_;
  bool first_msg_;
  rclcpp::Time last_cmd_vel_time_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NewOdomNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
