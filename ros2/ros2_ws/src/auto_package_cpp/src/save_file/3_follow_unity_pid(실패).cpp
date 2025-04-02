/*
 * Path Following Node
 * 
 * 기능:
 * - Pure Pursuit 알고리즘 기반 경로 추적
 * - 동적 Look-ahead Distance 조절
 * - PID 제어를 통한 부드러운 속도 제어
 * 
 * 토픽:
 * - 구독:
 *   - /local_path: 추적할 지역 경로
 *   - /odom: 로봇의 위치 정보
 * - 발행:
 *   - /cmd_vel: 로봇 제어 명령
 */

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace std::chrono_literals;

class PathFollower : public rclcpp::Node {
public:
    PathFollower() : Node("path_follower") {
        // 파라미터 초기화 - 속도 관련 값 대폭 증가
        max_linear_vel_ = 3.6;   // 최대 선속도 크게 증가 (0.8 -> 3.6 m/s)
        max_angular_vel_ = 3.0;  // 최대 각속도를 2.0에서 3.0으로 증가
        min_look_ahead_ = 0.3;   // 최소 전방주시거리 감소 (0.5 -> 0.3 m)
        max_look_ahead_ = 1.5;   // 최대 전방주시거리 조정 (2.0 -> 1.5 m)
        goal_tolerance_ = 0.2;   // 목표점 도달 허용 오차

        // PID 제어 게인 크게 증가
        kp_linear_ = 2.0;    // 선속도 비례게인 크게 증가 (0.5 -> 2.0)
        ki_linear_ = 0.3;    // 선속도 적분게인 증가 (0.1 -> 0.3)
        kd_linear_ = 0.2;    // 선속도 미분게인 증가 (0.1 -> 0.2)
        kp_angular_ = 2.0;   // 각속도 비례게인 증가 (1.0 -> 2.0)
        ki_angular_ = 0.3;   // 각속도 적분게인 증가 (0.1 -> 0.3)
        kd_angular_ = 0.3;   // 각속도 미분게인 증가 (0.1 -> 0.3)

        // 에러 누적값 초기화
        linear_error_sum_ = 0.0;
        angular_error_sum_ = 0.0;
        last_linear_error_ = 0.0;
        last_angular_error_ = 0.0;

        // 적분 제한값 설정
        max_integral_sum_ = 2.0;
        
        // Publishers
        cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        
        // Subscribers
        path_sub_ = create_subscription<nav_msgs::msg::Path>(
            "/local_path", 10,
            std::bind(&PathFollower::path_callback, this, std::placeholders::_1));

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, 
            std::bind(&PathFollower::odom_callback, this, std::placeholders::_1));

        // 목표점 구독자 추가
        goal_sub_ = create_subscription<geometry_msgs::msg::Point>(
            "/goal_point", 10,
            std::bind(&PathFollower::goal_callback, this, std::placeholders::_1));

        // 도착 판정 거리 설정
        arrival_threshold_ = 0.3;  // 30cm
        has_goal_ = false;

        // 제어 타이머 (50Hz)
        timer_ = create_wall_timer(20ms, std::bind(&PathFollower::control_loop, this));

        RCLCPP_INFO(get_logger(), "Path Follower initialized");
    }

private:
    // 상태 열거형 추가
    enum class DriveState {
        NORMAL,         // 일반 주행
        ROTATING,       // 회전 중
        APPROACHING    // 경로 접근 중
    };
    
    void path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
        current_path_ = *msg;
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_pose_ = msg->pose.pose;
        current_twist_ = msg->twist.twist;

        // 현재 방향(yaw) 계산
        tf2::Quaternion q(
            current_pose_.orientation.x,
            current_pose_.orientation.y,
            current_pose_.orientation.z,
            current_pose_.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch;
        m.getRPY(roll, pitch, current_yaw_);
    }

    void goal_callback(const geometry_msgs::msg::Point::SharedPtr msg) {
        current_goal_ = *msg;
        has_goal_ = true;
        RCLCPP_INFO(get_logger(), "Received new goal: (%.2f, %.2f)", msg->x, msg->y);
    }

    void control_loop() {
        if (!has_goal_) {
            stop_robot();
            return;
        }

        // 현재 위치와 목표점 사이의 거리 계산 - 가장 먼저 체크
        double goal_distance = std::hypot(
            current_goal_.x - current_pose_.position.x,
            current_goal_.y - current_pose_.position.y
        );

        // 목표점 도달 확인을 최우선으로 처리
        if (goal_distance <= arrival_threshold_) {
            RCLCPP_INFO(get_logger(), "Arrived at goal point!");
            stop_robot();
            has_goal_ = false;  // 새로운 목표점을 기다림
            current_state_ = DriveState::NORMAL;  // 상태 초기화
            return;  // 다른 처리 중단
        }

        // 경로가 비어있는 경우 정지
        if (current_path_.poses.empty()) {
            RCLCPP_WARN(get_logger(), "No valid path available, stopping robot");
            stop_robot();
            return;
        }

        // 1. 현재 위치에서 가장 가까운 경로점 찾기
        size_t closest_idx = find_closest_point();
        double path_distance = calculate_path_distance(closest_idx);
        
        // 상태에 따른 제어
        switch (current_state_) {
            case DriveState::NORMAL:
                if (path_distance > path_threshold_) {
                    // 경로 이탈 감지
                    RCLCPP_INFO(get_logger(), "Path deviation detected. Switching to rotation mode");
                    current_state_ = DriveState::ROTATING;
                    stop_robot();
                    return;
                }
                normal_drive(closest_idx);
                break;
                
            case DriveState::ROTATING:
                if (align_with_path(closest_idx)) {
                    RCLCPP_INFO(get_logger(), "Aligned with path. Switching to approach mode");
                    current_state_ = DriveState::APPROACHING;
                }
                break;
                
            case DriveState::APPROACHING:
                if (path_distance > path_threshold_) {
                    // 접근 중 다시 이탈
                    RCLCPP_INFO(get_logger(), "Path deviation during approach. Switching to rotation mode");
                    current_state_ = DriveState::ROTATING;
                    stop_robot();
                    return;
                }
                if (path_distance < path_threshold_ * 0.5) {
                    // 경로에 충분히 가까워짐
                    RCLCPP_INFO(get_logger(), "Successfully approached path. Switching to normal mode");
                    current_state_ = DriveState::NORMAL;
                    return;
                }
                approach_path();
                break;
        }
    }

    double calculate_path_distance(size_t closest_idx) {
        double dx = current_path_.poses[closest_idx].pose.position.x - current_pose_.position.x;
        double dy = current_path_.poses[closest_idx].pose.position.y - current_pose_.position.y;
        return std::hypot(dx, dy);
    }

    void normal_drive(size_t closest_idx) {
        // 기존의 일반 주행 로직
        double current_vel = std::hypot(current_twist_.linear.x, current_twist_.linear.y);
        double look_ahead = std::min(std::max(current_vel * 1.0, min_look_ahead_), max_look_ahead_);

        geometry_msgs::msg::Point target_point;
        if (find_look_ahead_point(closest_idx, look_ahead, target_point)) {
            calculate_and_publish_cmd_vel(target_point, look_ahead);
        }
    }

    bool align_with_path(size_t closest_idx) {
        // 경로 방향 계산
        double path_direction = calculate_path_direction(closest_idx);
        
        // 현재 방향과 경로 방향의 차이 계산
        double angle_error = path_direction - current_yaw_;
        while (angle_error > M_PI) angle_error -= 2 * M_PI;
        while (angle_error < -M_PI) angle_error += 2 * M_PI;

        // 각도 오차가 작으면 정렬 완료
        if (std::abs(angle_error) < 0.1) {  // 약 5.7도
            return true;
        }

        // 회전만 수행
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.y = (angle_error > 0) ? 2.5 : -2.5;  // z에서 y로 변경
        
        // 최대 각속도 제한 (3.0 rad/s)
        cmd_vel.angular.y = std::min(std::max(cmd_vel.angular.y, -max_angular_vel_), max_angular_vel_);
        
        cmd_vel_pub_->publish(cmd_vel);
        
        return false;
    }

    void approach_path() {
        // 전진만 수행
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = 0.8;
        cmd_vel.angular.y = 0.0;  // z에서 y로 변경
        cmd_vel_pub_->publish(cmd_vel);
    }

    double calculate_path_direction(size_t closest_idx) {
        // 경로의 방향 계산
        size_t next_idx = std::min(closest_idx + 1, current_path_.poses.size() - 1);
        double dx = current_path_.poses[next_idx].pose.position.x - 
                   current_path_.poses[closest_idx].pose.position.x;
        double dy = current_path_.poses[next_idx].pose.position.y - 
                   current_path_.poses[closest_idx].pose.position.y;
        return std::atan2(dy, dx);
    }

    size_t find_closest_point() {
        size_t closest_idx = 0;
        double min_dist = std::numeric_limits<double>::max();

        for (size_t i = 0; i < current_path_.poses.size(); ++i) {
            double dx = current_path_.poses[i].pose.position.x - current_pose_.position.x;
            double dy = current_path_.poses[i].pose.position.y - current_pose_.position.y;
            double dist = std::hypot(dx, dy);

            if (dist < min_dist) {
                min_dist = dist;
                closest_idx = i;
            }
        }

        return closest_idx;
    }

    bool find_look_ahead_point(
        size_t start_idx, double look_ahead, geometry_msgs::msg::Point& target_point) {
        
        // 로봇의 현재 진행 방향 계산
        double robot_heading = current_yaw_;
        
        // 가장 적절한 look-ahead point 찾기
        size_t best_idx = start_idx;
        double min_angle_diff = M_PI;
        
        for (size_t i = start_idx; i < current_path_.poses.size(); ++i) {
            double dx = current_path_.poses[i].pose.position.x - current_pose_.position.x;
            double dy = current_path_.poses[i].pose.position.y - current_pose_.position.y;
            double distance = std::hypot(dx, dy);
            
            if (distance >= look_ahead * 0.8 && distance <= look_ahead * 1.2) {
                // 해당 포인트까지의 각도 계산
                double point_angle = std::atan2(dy, dx);
                double angle_diff = std::abs(point_angle - robot_heading);
                while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
                
                // 진행 방향과 가장 잘 맞는 포인트 선택
                if (angle_diff < min_angle_diff) {
                    min_angle_diff = angle_diff;
                    best_idx = i;
                }
            }
        }
        
        target_point = current_path_.poses[best_idx].pose.position;
        return true;
    }

    void calculate_and_publish_cmd_vel(
        const geometry_msgs::msg::Point& target_point, double look_ahead) {
        
        // 1. 로봇 좌표계에서의 목표점 위치 계산
        double dx = target_point.x - current_pose_.position.x;
        double dy = target_point.y - current_pose_.position.y;
        
        // 2. 목표점까지의 거리와 각도 계산
        double distance = std::hypot(dx, dy);
        double target_angle = std::atan2(dy, dx);
        
        // 3. 각도 오차 계산 (로봇 기준)
        double angle_error = target_angle - current_yaw_;
        while (angle_error > M_PI) angle_error -= 2 * M_PI;
        while (angle_error < -M_PI) angle_error += 2 * M_PI;

        // 4. PID 제어
        double base_speed = 1.2;
        double linear_vel = base_speed;
        
        // 각도 오차가 큰 경우 회전 우선
        if (std::abs(angle_error) > M_PI/4) {
            linear_vel = base_speed * 0.7;
        }
        
        // 각속도 계산 (부호 반전 제거)
        double angular_vel = kp_angular_ * angle_error +
                            ki_angular_ * angular_error_sum_ +
                            kd_angular_ * (angle_error - last_angular_error_) / 0.02;
        
        // 5. 속도 제한
        linear_vel = std::min(std::max(linear_vel, 0.7), max_linear_vel_);
        angular_vel = std::min(std::max(angular_vel, -max_angular_vel_), max_angular_vel_);

        // 선속도 PID에서 적분항 강화
        linear_error_sum_ += linear_vel * 0.02;
        linear_error_sum_ = std::min(std::max(linear_error_sum_, -max_integral_sum_), max_integral_sum_);
        
        // 각속도 PID에서 적분항 강화
        angular_error_sum_ += angle_error * 0.02;
        angular_error_sum_ = std::min(std::max(angular_error_sum_, -max_integral_sum_), max_integral_sum_);

        // 6. 제어 명령 발행
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = linear_vel;
        cmd_vel.angular.y = angular_vel;  // z에서 y로 변경
        cmd_vel_pub_->publish(cmd_vel);

        // 에러값 저장
        last_angular_error_ = angle_error;
    }

    void stop_robot() {
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.y = 0.0;  // z에서 y로 변경
        cmd_vel_pub_->publish(cmd_vel);
        
        // 정지 시 에러 누적값 초기화
        linear_error_sum_ = 0.0;
        angular_error_sum_ = 0.0;
    }

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    
    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr goal_sub_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr timer_;
    
    // 현재 상태
    nav_msgs::msg::Path current_path_;
    geometry_msgs::msg::Pose current_pose_;
    geometry_msgs::msg::Twist current_twist_;
    double current_yaw_;

    // 파라미터
    double max_linear_vel_;
    double max_angular_vel_;
    double min_look_ahead_;
    double max_look_ahead_;
    double goal_tolerance_;

    // PID 제어 관련 변수
    double kp_linear_, ki_linear_, kd_linear_;
    double kp_angular_, ki_angular_, kd_angular_;
    double linear_error_sum_, angular_error_sum_;
    double last_linear_error_, last_angular_error_;

    // 추가된 멤버 변수
    geometry_msgs::msg::Point current_goal_;
    bool has_goal_;
    double arrival_threshold_;

    // 적분 제한값 설정
    double max_integral_sum_;

    // 추가 멤버 변수
    DriveState current_state_ = DriveState::NORMAL;
    double path_threshold_ = 1.0;  // 경로 이탈 판단 거리 (미터)
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathFollower>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 