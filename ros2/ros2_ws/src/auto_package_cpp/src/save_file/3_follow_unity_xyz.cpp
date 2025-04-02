/*
 * Unity Robot Controller Node
 * 
 * 기능:
 * - local_path를 따라 로봇을 제어
 * - 3축 선속도(x,y,z)와 y축 회전 제어
 * - Unity 환경에 맞춘 좌표계 변환
 * 
 * 토픽:
 * - 구독:
 *   - /local_path: 지역 경로
 *   - /odom: 로봇의 위치 정보
 * - 발행:
 *   - /cmd_vel: 로봇 제어 명령
 */

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>

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
                       look_ahead_dist_(0.5),    // 전방 주시 거리
                       max_linear_speed_(1.0),   // 최대 선속도
                       min_linear_speed_(0.1),   // 최소 선속도
                       max_angular_speed_(1.0),  // 최대 각속도
                       goal_tolerance_(0.1)      // 목표점 도달 허용 오차
    {
        // Publishers
        cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        // Subscribers
        path_sub_ = create_subscription<nav_msgs::msg::Path>(
            "local_path", 10,
            std::bind(&UnityController::path_callback, this, std::placeholders::_1));

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10,
            std::bind(&UnityController::odom_callback, this, std::placeholders::_1));

        // Control loop timer
        timer_ = create_wall_timer(50ms, std::bind(&UnityController::control_loop, this));

        RCLCPP_INFO(get_logger(), "Unity Controller initialized");
    }

private:
    void path_callback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        path_ = *msg;
        has_path_ = !msg->poses.empty();
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
        current_yaw_ = yaw - M_PI;  // Unity 좌표계 보정
    }

    void control_loop()
    {
        if (!has_path_ || !has_odom_) {
            stop_robot();
            return;
        }

        // 현재 로봇 위치
        double robot_x = odom_.pose.pose.position.x;
        double robot_y = odom_.pose.pose.position.y;

        // 목표점 찾기
        geometry_msgs::msg::Point target_point;
        if (!find_target_point(robot_x, robot_y, target_point)) {
            stop_robot();
            return;
        }

        // 로봇 기준 목표점 좌표 계산
        double dx = target_point.x - robot_x;
        double dy = target_point.y - robot_y;

        // Unity 좌표계에 맞춰 변환
        double target_angle = std::atan2(dy, dx) - M_PI;
        double angle_diff = normalize_angle(target_angle - current_yaw_);

        // 속도 명령 생성
        geometry_msgs::msg::Twist cmd_vel;
        
        // 거리에 비례한 선속도 계산
        double distance = std::sqrt(dx*dx + dy*dy);
        double speed = std::min(max_linear_speed_, distance);
        speed = std::max(min_linear_speed_, speed);

        // 3축 선속도 분해
        cmd_vel.linear.x = speed * std::cos(angle_diff);
        cmd_vel.linear.y = speed * std::sin(angle_diff);
        cmd_vel.linear.z = 0.0;  // 높이 제어는 하지 않음

        // y축 회전 제어 (항상 정면 유지)
        cmd_vel.angular.y = angle_diff * max_angular_speed_;

        // 명령 발행
        cmd_vel_pub_->publish(cmd_vel);
    }

    bool find_target_point(double robot_x, double robot_y, geometry_msgs::msg::Point& target)
    {
        if (path_.poses.empty()) return false;

        // 경로 끝점까지의 거리 확인
        double dist_to_end = std::hypot(
            path_.poses.back().pose.position.x - robot_x,
            path_.poses.back().pose.position.y - robot_y
        );

        // 목표점에 도달했는지 확인
        if (dist_to_end < goal_tolerance_) {
            return false;
        }

        // 전방 주시 거리의 목표점 찾기
        for (size_t i = 0; i < path_.poses.size() - 1; ++i) {
            double dist = std::hypot(
                path_.poses[i].pose.position.x - robot_x,
                path_.poses[i].pose.position.y - robot_y
            );

            if (dist >= look_ahead_dist_) {
                target = path_.poses[i].pose.position;
                return true;
            }
        }

        // 전방 주시 거리 내에 점이 없으면 마지막 점 사용
        target = path_.poses.back().pose.position;
        return true;
    }

    void stop_robot()
    {
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
        cmd_vel.linear.z = 0.0;
        cmd_vel.angular.y = 0.0;
        cmd_vel_pub_->publish(cmd_vel);
    }

    double normalize_angle(double angle)
    {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Data
    nav_msgs::msg::Path path_;
    nav_msgs::msg::Odometry odom_;
    double current_yaw_;

    // Parameters
    double look_ahead_dist_;
    double max_linear_speed_;
    double min_linear_speed_;
    double max_angular_speed_;
    double goal_tolerance_;

    // State flags
    bool has_path_;
    bool has_odom_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UnityController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 