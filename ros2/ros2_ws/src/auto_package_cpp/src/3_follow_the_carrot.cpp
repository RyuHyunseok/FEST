#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "ssafy_msgs/msg/turtlebot_status.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <vector>
#include <algorithm>

// M_PI 정의 추가
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace std::chrono_literals;

class FollowTheCarrot : public rclcpp::Node
{
public:
    FollowTheCarrot() : Node("path_tracking"), 
                        is_odom_(false), 
                        is_path_(false), 
                        is_status_(false),
                        robot_yaw_(0.0),
                        lfd_(0.1),
                        min_lfd_(0.3),
                        max_lfd_(3.0) {
        
        // Publisher
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        
        // Subscribers
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, 
            std::bind(&FollowTheCarrot::odom_callback, this, std::placeholders::_1));
        
        status_sub_ = this->create_subscription<ssafy_msgs::msg::TurtlebotStatus>(
            "/turtlebot_status", 10,
            std::bind(&FollowTheCarrot::status_callback, this, std::placeholders::_1));
        
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/local_path", 10,
            std::bind(&FollowTheCarrot::path_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(50ms, std::bind(&FollowTheCarrot::timer_callback, this));
    }

private:
    void timer_callback()
    {
        if (is_status_ && is_odom_ && is_path_) {
            if (path_msg_.poses.size() > 1) {
                bool is_look_forward_point = false;
                
                double robot_pose_x = odom_msg_.pose.pose.position.x;
                double robot_pose_y = odom_msg_.pose.pose.position.y;
                
                // 현재 위치와 경로 시작점 사이의 거리 계산
                double lateral_error = std::sqrt(
                    std::pow(path_msg_.poses[0].pose.position.x - robot_pose_x, 2) + 
                    std::pow(path_msg_.poses[0].pose.position.y - robot_pose_y, 2));
                
                // Look Forward Distance 동적 조정
                lfd_ = (status_msg_.twist.linear.x + lateral_error) * 1.0;
                lfd_ = std::max(min_lfd_, std::min(lfd_, max_lfd_));
                
                RCLCPP_INFO(this->get_logger(), "LFD: %f", lfd_);

                // Look Forward Point 찾기
                double min_dis = std::numeric_limits<double>::infinity();
                geometry_msgs::msg::Point forward_point;
                geometry_msgs::msg::Point current_point;

                for (const auto& pose : path_msg_.poses) {
                    current_point = pose.pose.position;
                    
                    double dis = std::sqrt(
                        std::pow(path_msg_.poses[0].pose.position.x - current_point.x, 2) + 
                        std::pow(path_msg_.poses[0].pose.position.y - current_point.y, 2));
                    
                    if (std::abs(dis - lfd_) < min_dis) {
                        min_dis = std::abs(dis - lfd_);
                        forward_point = current_point;
                        is_look_forward_point = true;
                    }
                }

                if (is_look_forward_point) {
                    // 전역 좌표계의 전방 포인트를 로봇 좌표계로 변환
                    std::vector<double> global_forward_point = {forward_point.x, forward_point.y, 1.0};
                    
                    double cos_yaw = std::cos(robot_yaw_);
                    double sin_yaw = std::sin(robot_yaw_);
                    
                    // 역변환 행렬 계산 (전역 -> 로컬)
                    std::vector<std::vector<double>> det_trans_matrix = {
                        {cos_yaw, sin_yaw, -robot_pose_x * cos_yaw - robot_pose_y * sin_yaw},
                        {-sin_yaw, cos_yaw, robot_pose_x * sin_yaw - robot_pose_y * cos_yaw},
                        {0, 0, 1}
                    };
                    
                    // 로봇 좌표계에서의 전방 포인트 계산
                    std::vector<double> local_forward_point(3);
                    for (int i = 0; i < 3; i++) {
                        local_forward_point[i] = 0;
                        for (int j = 0; j < 3; j++) {
                            local_forward_point[i] += det_trans_matrix[i][j] * global_forward_point[j];
                        }
                    }
                    
                    // 조향각 계산
                    double theta = -std::atan2(local_forward_point[1], local_forward_point[0]);
                    
                    // 회전각에 따른 속도 조절
                    double abs_theta = std::abs(theta);
                    double base_speed = 1.0;  // 기본 속도
                    double min_speed = 0.1;   // 최소 속도
                    
                    // 회전각이 클수록 속도 감소
                    double speed_factor = 1.0 - (abs_theta / M_PI) * 0.6;  // 감속 강도를 0.8에서 0.6으로 줄임
                    speed_factor = std::max(0.2, speed_factor);  // 최소 속도를 0.1에서 0.2로 증가
                    double smooth_factor = 0.7;  // 속도 변화를 부드럽게 하기 위한 계수
                    static double prev_speed = 0.0;
                    cmd_msg_.linear.x = (base_speed * speed_factor * smooth_factor) + 
                                        (prev_speed * (1.0 - smooth_factor));
                    prev_speed = cmd_msg_.linear.x;
                    
                    // 각속도 게인을 줄여 급격한 회전 방지
                    double angular_gain = 1.5;  // 2.0에서 감소
                    cmd_msg_.angular.z = theta * angular_gain;
                    
                    RCLCPP_INFO(this->get_logger(), 
                        "theta: %f, speed: %f", 
                        theta, 
                        cmd_msg_.linear.x);
                }
            } else {
                RCLCPP_INFO(this->get_logger(), "No path points available");
                cmd_msg_.linear.x = 0.0;
                cmd_msg_.angular.z = 0.0;
            }

            cmd_pub_->publish(cmd_msg_);
        }
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        is_odom_ = true;
        odom_msg_ = *msg;
        
        // 쿼터니언에서 오일러 각도로 변환
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        robot_yaw_ = yaw;
    }

    void path_callback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        is_path_ = true;
        path_msg_ = *msg;
    }

    void status_callback(const ssafy_msgs::msg::TurtlebotStatus::SharedPtr msg)
    {
        is_status_ = true;
        status_msg_ = *msg;
    }

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    
    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<ssafy_msgs::msg::TurtlebotStatus>::SharedPtr status_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Messages
    nav_msgs::msg::Odometry odom_msg_;
    nav_msgs::msg::Path path_msg_;
    geometry_msgs::msg::Twist cmd_msg_;
    ssafy_msgs::msg::TurtlebotStatus status_msg_;
    
    // State flags
    bool is_odom_;
    bool is_path_;
    bool is_status_;
    
    // Parameters
    double robot_yaw_;
    double lfd_;
    double min_lfd_;
    double max_lfd_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FollowTheCarrot>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 