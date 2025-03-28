/*
 * Follow The Carrot Path Tracking Node
 * 
 * 기능:
 * - Pure Pursuit 알고리즘을 사용한 경로 추적
 * - 동적 Look Forward Distance 조정
 * - 목표점 도달 감지 및 정지
 * 
 * 토픽:
 * - 구독:
 *   - /local_path: 추적할 로컬 경로
 *   - /odom: 로봇의 위치 정보
 *   - /turtlebot_status: 로봇의 상태 정보
 * - 발행:
 *   - /cmd_vel: 로봇 제어 명령
 */

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

const double M_PI = std::acos(-1);

using namespace std::chrono_literals;

class FollowTheCarrot : public rclcpp::Node
{
public:
    FollowTheCarrot() : Node("path_tracking"), 
                        robot_yaw_(0.0),
                        lfd_(0.1),
                        min_lfd_(0.3),
                        max_lfd_(3.0),
                        destination_radius_(0.3) { // 목적지 도착 반경 30cm
        
        // Publishers
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
        if (path_msg_.poses.size() > 1) {
            bool is_look_forward_point = false;
            
            double robot_pose_x = odom_msg_.pose.pose.position.x;
            double robot_pose_y = odom_msg_.pose.pose.position.y;
            
            // 목적지까지의 거리 계산
            geometry_msgs::msg::Point destination = path_msg_.poses.back().pose.position;
            double dist_to_destination = std::sqrt(
                std::pow(destination.x - robot_pose_x, 2) + 
                std::pow(destination.y - robot_pose_y, 2));
            
            // 목적지에 도착했는지 확인
            if (dist_to_destination <= destination_radius_) {
                // 목적지 도착 시 정지
                cmd_msg_.linear.x = 0.0;
                cmd_msg_.angular.z = 0.0;
                cmd_pub_->publish(cmd_msg_);
            }
            else {
                // 현재 위치와 경로 시작점 사이의 거리 계산
                double lateral_error = std::sqrt(
                    std::pow(path_msg_.poses[0].pose.position.x - robot_pose_x, 2) + 
                    std::pow(path_msg_.poses[0].pose.position.y - robot_pose_y, 2));
                
                // Look Forward Distance 동적 조정
                lfd_ = (status_msg_.twist.linear.x + lateral_error) * 0.7;  // 계수를 1.0에서 0.7로 감소
                lfd_ = std::max(min_lfd_, std::min(lfd_, max_lfd_));

                // Look Forward Point 찾기
                geometry_msgs::msg::Point forward_point;
                geometry_msgs::msg::Point current_point;
                
                // 로봇과 가장 가까운 경로점 찾기
                size_t closest_idx = 0;
                double min_path_dis = std::numeric_limits<double>::infinity();
                
                for (size_t i = 0; i < path_msg_.poses.size(); i++) {
                    double dx = path_msg_.poses[i].pose.position.x - robot_pose_x;
                    double dy = path_msg_.poses[i].pose.position.y - robot_pose_y;
                    double dist = std::sqrt(dx*dx + dy*dy);
                    
                    if (dist < min_path_dis) {
                        min_path_dis = dist;
                        closest_idx = i;
                    }
                }
                
                // 가장 가까운 점에서부터 LFD만큼 떨어진 점 찾기
                double current_distance = 0.0;
                is_look_forward_point = false;
                
                for (size_t i = closest_idx; i < path_msg_.poses.size()-1; i++) {
                    current_point = path_msg_.poses[i].pose.position;
                    geometry_msgs::msg::Point next_point = path_msg_.poses[i+1].pose.position;
                    
                    double segment_dx = next_point.x - current_point.x;
                    double segment_dy = next_point.y - current_point.y;
                    double segment_length = std::sqrt(segment_dx*segment_dx + segment_dy*segment_dy);
                    
                    // 세그먼트를 따라 이동하면서 LFD 거리에 도달하는지 확인
                    if (current_distance + segment_length > lfd_) {
                        // 세그먼트 내에서 LFD 지점 계산
                        double ratio = (lfd_ - current_distance) / segment_length;
                        forward_point.x = current_point.x + ratio * segment_dx;
                        forward_point.y = current_point.y + ratio * segment_dy;
                        is_look_forward_point = true;
                        break;
                    }
                    
                    current_distance += segment_length;
                }
                
                // 목표점이 발견되지 않았고 경로가 남아있다면 마지막 경로점 사용
                if (!is_look_forward_point && path_msg_.poses.size() > 0) {
                    forward_point = path_msg_.poses.back().pose.position;
                    is_look_forward_point = true;
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
                    double base_speed = 0.8;  // 기본 속도를 1.0에서 0.8로 감소
                    
                    // 회전각이 클수록 속도 감소 - 더 민감하게 속도 줄임
                    double speed_factor = 1.0 - (abs_theta / M_PI) * 0.8;  // 감속 강도를 0.6에서 0.8로 증가
                    speed_factor = std::max(0.15, speed_factor);  // 최소 속도를 0.2에서 0.15로 감소
                    double smooth_factor = 0.5;  // 속도 변화를 더 부드럽게 (0.7에서 0.5로 조정)
                    static double prev_speed = 0.0;
                    cmd_msg_.linear.x = (base_speed * speed_factor * smooth_factor) + 
                                        (prev_speed * (1.0 - smooth_factor));
                    prev_speed = cmd_msg_.linear.x;
                    
                    // 각속도 게인을 조정하여 급격한 회전 방지
                    double angular_gain = 1.0;  // 1.5에서 1.0으로 감소
                    cmd_msg_.angular.z = theta * angular_gain;

                    // 급격한 회전 시 더 일찍 멈추고 회전만 수행
                    double angular_threshold = M_PI / 2;  // 임계값 설정 (3π/4에서 π/2로 줄임)
                    if (abs_theta > angular_threshold) {
                        cmd_msg_.linear.x = 0.0;  // 선속도 0으로 설정
                    }
                }
            }
        } else {
            cmd_msg_.linear.x = 0.0;
            cmd_msg_.angular.z = 0.0;
        }

        cmd_pub_->publish(cmd_msg_);
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
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
        path_msg_ = *msg;
    }

    void status_callback(const ssafy_msgs::msg::TurtlebotStatus::SharedPtr msg)
    {
        status_msg_ = *msg;
    }

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    
    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<ssafy_msgs::msg::TurtlebotStatus>::SharedPtr status_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    
    // Timers
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Messages
    nav_msgs::msg::Odometry odom_msg_;
    nav_msgs::msg::Path path_msg_;
    geometry_msgs::msg::Twist cmd_msg_;
    ssafy_msgs::msg::TurtlebotStatus status_msg_;
    
    // Parameters
    double robot_yaw_;
    double lfd_;
    double min_lfd_;
    double max_lfd_;
    double destination_radius_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FollowTheCarrot>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 