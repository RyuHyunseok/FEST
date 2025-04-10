/*
 * Unity Robot Controller Node
 * 
 * 1. 기능:
 *    - local_path를 따라 로봇 제어
 *    - Unity 환경에 맞춘 좌표계 변환
 *    - 목표점 도달 확인 및 속도 제어
 * 
 * 2. 구독 토픽:
 *    - /local_path: 지역 경로
 *    - /odom: 로봇의 위치 정보
 *    - /goal_point: 목표점 정보
 * 
 * 3. 발행 토픽:
 *    - /cmd_vel: 로봇 제어 명령
 *    - /goal_reached: 목표 도달 상태
 * 
 * 4. 사용 파일: 없음
 * 
 * 5. 저장 파일: 없음
 * 
 * 6. 주요 함수:
 *    - control_loop(): 로봇 제어 루프
 *    - find_next_point(): 다음 목표점 선택
 *    - normalize_angle(): 각도 정규화
 */

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <std_msgs/msg/bool.hpp>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace std::chrono_literals;

class UnityController : public rclcpp::Node 
{
public:
    UnityController() : Node("unity_controller"), 
                       has_path_(false),
                       has_odom_(false),
                       has_goal_(false),
                       is_goal_reached_(false),
                       linear_speed_(2.5),    // 선속도를 1.0으로 수정
                       max_angular_speed_(90.0),  // 최대 각속도를 180도/초로 증가
                       goal_tolerance_(5.0),      // 목표점 도달 허용 오차 (0.3m)
                       path_point_selection_(PATH_INDEX),  // 기본값으로 마지막 점 선택
                       path_point_index_(10)  // 기본 인덱스
    {
        // Publishers
        cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        goal_reached_pub_ = create_publisher<std_msgs::msg::Bool>("goal_reached", 10);

        // Subscribers
        path_sub_ = create_subscription<nav_msgs::msg::Path>(
            "local_path", 10,
            std::bind(&UnityController::path_callback, this, std::placeholders::_1));

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10,
            std::bind(&UnityController::odom_callback, this, std::placeholders::_1));

        goal_sub_ = create_subscription<geometry_msgs::msg::Point>(
            "/goal_point", 10,
            std::bind(&UnityController::goal_callback, this, std::placeholders::_1));

        // Control loop timer
        timer_ = create_wall_timer(50ms, std::bind(&UnityController::control_loop, this));

        // RCLCPP_INFO(get_logger(), "Unity Controller initialized");
    }

private:
    // 각도 변환을 위한 상수 추가 (클래스 내 private 섹션 상단에)
    const double RAD_TO_DEG = 180.0 / M_PI;
    const double DEG_TO_RAD = M_PI / 180.0;

    // 경로점 선택 방식을 위한 열거형 추가
    enum PathPointSelection {
        PATH_FIRST,      // 첫 번째 점
        PATH_LAST,       // 마지막 점
        PATH_FARTHEST,   // 가장 먼 점
        PATH_NEAREST,    // 가장 가까운 점
        PATH_INDEX       // 특정 인덱스의 점
    };
    PathPointSelection path_point_selection_;
    size_t path_point_index_;  // 선택할 경로점의 인덱스

    void path_callback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        path_ = *msg;
        has_path_ = !msg->poses.empty();
        
        if (!has_path_) {
            // RCLCPP_WARN(get_logger(), "Received empty path");
            stop_robot();
        }
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        odom_ = *msg;
        has_odom_ = true;

        // Unity의 180도 회전된 좌표계 고려
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        
        // Unity 좌표계에서는 방향이 반대로 해석되어야 함
        current_yaw_ = normalize_angle(-yaw * RAD_TO_DEG);  // 부호를 반대로 변경
        
        // RCLCPP_INFO(get_logger(), "Current yaw (degrees): %.2f", current_yaw_);
    }

    void goal_callback(const geometry_msgs::msg::Point::SharedPtr msg)
    {
        goal_point_ = *msg;
        has_goal_ = true;
        // RCLCPP_INFO(get_logger(), "Received new goal point: (%.2f, %.2f)", msg->x, msg->z);
    }

    void control_loop()
    {
        if (!has_path_ || !has_odom_ || !has_goal_) {
            stop_robot();
            return;
        }

        // 현재 로봇 위치
        double robot_x = odom_.pose.pose.position.x;
        double robot_y = odom_.pose.pose.position.y;

        // 목표점까지의 거리 확인
        double dist_to_goal = std::hypot(
            goal_point_.x - robot_x,
            goal_point_.y - robot_y
        );

        // RCLCPP_INFO(get_logger(), 
        //     "Distance to goal: %.2f, Goal(x,z): (%.2f, %.2f), Robot(x,z): (%.2f, %.2f)",
        //     dist_to_goal,
        //     goal_point_.x, goal_point_.y,
        //     robot_x, robot_y
        // );

        // 목표점 도달 상태 확인 및 발행
        bool current_goal_status = (dist_to_goal < goal_tolerance_);
        
        // 상태가 변경되었을 때만 발행
        if (current_goal_status != is_goal_reached_) {
            is_goal_reached_ = current_goal_status;
            auto msg = std_msgs::msg::Bool();
            msg.data = is_goal_reached_;
            goal_reached_pub_->publish(msg);
            
            if (is_goal_reached_) {
                // RCLCPP_INFO(get_logger(), "Goal reached!");
            } else {
                // RCLCPP_INFO(get_logger(), "Moving away from goal");
            }
        }

        if (is_goal_reached_) {
            stop_robot();
            return;
        }

        // 다음 경로점 찾기
        geometry_msgs::msg::Point target_point;
        if (!find_next_point(robot_x, robot_y, target_point)) {
            // RCLCPP_WARN(get_logger(), "No valid path point found");
            stop_robot();
            return;
        }

        // 로봇 기준 목표점 좌표 계산 (x,z 평면)
        double dx = target_point.x - robot_x;
        double dy = target_point.y - robot_y;

        // 이동 방향 각도 계산 (기존 코드 유지)
        double movement_direction = std::atan2(dy, dx) * RAD_TO_DEG;
        
        // 로봇의 현재 방향과 이동 방향의 차이 계산 (y축 방향 반전)
        double angle_diff = normalize_angle(-movement_direction - current_yaw_);  // movement_direction에 음수를 취함
        
        // 각도 차이가 ±10도 이내면 회전하지 않음
        double dead_zone = 10.0;
        double angular_speed = 0.0;
        
        if (std::abs(angle_diff) > dead_zone) {
            double p_gain = 0.5;
            angular_speed = std::min(std::max(angle_diff * p_gain, -max_angular_speed_), max_angular_speed_);
        }

        // 속도 명령 생성
        geometry_msgs::msg::Twist cmd_vel;
        
        // 목표 방향으로 직접 이동 (로봇의 방향과 무관하게)
        cmd_vel.linear.x = -linear_speed_ * std::sin(movement_direction * DEG_TO_RAD);
        cmd_vel.linear.y = 0.0;
        cmd_vel.linear.z = linear_speed_ * std::cos(movement_direction * DEG_TO_RAD);
        
        cmd_vel.angular.y = angular_speed;

        // 명령 발행
        cmd_vel_pub_->publish(cmd_vel);
    }

    bool find_next_point(double robot_x, double robot_y, geometry_msgs::msg::Point& target)
    {
        if (path_.poses.empty()) return false;

        size_t selected_idx = 0;

        switch (path_point_selection_) {
            case PATH_FIRST:
                selected_idx = 0;
                break;

            case PATH_LAST:
                selected_idx = path_.poses.size() - 1;
                break;

            case PATH_FARTHEST: {
                double max_dist = -1.0;
                for (size_t i = 0; i < path_.poses.size(); ++i) {
                    const auto& pose = path_.poses[i];
                    double dx = pose.pose.position.x - robot_x;
                    double dy = pose.pose.position.y - robot_y;
                    double dist = std::hypot(dx, dy);
                    if (dist > max_dist) {
                        max_dist = dist;
                        selected_idx = i;
                    }
                }
                break;
            }

            case PATH_NEAREST: {
                double min_dist = std::numeric_limits<double>::max();
                for (size_t i = 0; i < path_.poses.size(); ++i) {
                    const auto& pose = path_.poses[i];
                    double dx = pose.pose.position.x - robot_x;
                    double dy = pose.pose.position.y - robot_y;
                    double dist = std::hypot(dx, dy);
                    if (dist < min_dist) {
                        min_dist = dist;
                        selected_idx = i;
                    }
                }
                break;
            }

            case PATH_INDEX:
                // 인덱스가 유효한지 확인
                if (path_point_index_ < path_.poses.size()) {
                    selected_idx = path_point_index_;
                } else {
                    // 유효하지 않은 인덱스면 마지막 점 선택
                    selected_idx = path_.poses.size() - 1;
                }
                break;
        }

        // 선택된 점의 좌표를 target으로 설정
        target.x = path_.poses[selected_idx].pose.position.x;
        target.y = path_.poses[selected_idx].pose.position.y;
        return true;
    }

    void stop_robot()
    {
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = 0.0;  // 전진/후진 정지
        cmd_vel.linear.y = 0.0;  // 높이 방향
        cmd_vel.linear.z = 0.0;  // 좌우 이동 정지
        cmd_vel.angular.y = 0.0; // 회전 정지
        cmd_vel_pub_->publish(cmd_vel);
    }

    double normalize_angle(double angle)
    {
        while (angle > 180.0) angle -= 360.0;
        while (angle < -180.0) angle += 360.0;
        return angle;
    }

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr goal_reached_pub_;

    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr goal_sub_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Data
    nav_msgs::msg::Path path_;
    nav_msgs::msg::Odometry odom_;
    geometry_msgs::msg::Point goal_point_;
    double current_yaw_;

    // Parameters
    double linear_speed_;
    double max_angular_speed_;
    double goal_tolerance_;

    // State flags
    bool has_path_;
    bool has_odom_;
    bool has_goal_;
    bool is_goal_reached_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UnityController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 