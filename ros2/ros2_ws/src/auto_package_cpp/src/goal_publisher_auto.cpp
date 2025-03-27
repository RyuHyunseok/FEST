#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "auto_package_cpp/file_path.hpp"
#include <iostream>
#include <string>
#include <thread>
#include <fstream>
#include <vector>
#include <cmath>

// 전역 변수로 파일 경로 선언
const std::string PATH_FILE = auto_package_cpp::create_file_path("auto_package_cpp", "path/hoom2_path.txt");
const double GOAL_THRESHOLD = 0.35; // 목표점 도달 판단 거리 (미터)

class GoalPublisher : public rclcpp::Node {
public:
    GoalPublisher() : Node("goal_publisher") {
        // 목표점 발행자 생성
        goal_pub_ = create_publisher<geometry_msgs::msg::Point>("/goal_point", 10);
        
        // 로봇 위치 구독자 생성
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10,
            std::bind(&GoalPublisher::odom_callback, this, std::placeholders::_1));

        // 경로 파일 읽기
        loadPathFromFile();

        // 발행자 초기화를 기다리는 타이머 생성
        init_timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&GoalPublisher::init_callback, this));

        RCLCPP_INFO(get_logger(), "Goal Publisher initialized.");
        RCLCPP_INFO(get_logger(), "Publishing goals from path file: %s", PATH_FILE.c_str());
    }

private:
    void init_callback() {
        // 발행자가 초기화되었는지 확인
        if (goal_pub_->get_subscription_count() > 0) {
            // 초기 목표점 발행
            if (!path_points_.empty()) {
                goal_pub_->publish(path_points_[current_goal_index_]);
                RCLCPP_INFO(get_logger(), 
                    "Published initial goal point %zu/%zu: (%.2f, %.2f) meters", 
                    current_goal_index_ + 1, path_points_.size(), 
                    path_points_[current_goal_index_].x, 
                    path_points_[current_goal_index_].y);
            }
            // 타이머 중지
            init_timer_->cancel();
        }
    }

    void loadPathFromFile() {
        std::ifstream file(PATH_FILE);
        if (!file.is_open()) {
            RCLCPP_ERROR(get_logger(), "Failed to open path file: %s", PATH_FILE.c_str());
            return;
        }

        path_points_.clear();
        double x, y;
        while (file >> x >> y) {
            geometry_msgs::msg::Point point;
            point.x = x;
            point.y = y;
            point.z = 0.0;
            path_points_.push_back(point);
        }

        RCLCPP_INFO(get_logger(), "Loaded %zu points from path file", path_points_.size());
        file.close();
    }

    double calculateDistance(const geometry_msgs::msg::Point& point1, const geometry_msgs::msg::Point& point2) {
        double dx = point1.x - point2.x;
        double dy = point1.y - point2.y;
        return std::sqrt(dx*dx + dy*dy);
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        if (path_points_.empty()) {
            RCLCPP_WARN(get_logger(), "No path points available!");
            return;
        }

        // 현재 로봇 위치
        geometry_msgs::msg::Point current_pos;
        current_pos.x = msg->pose.pose.position.x;
        current_pos.y = msg->pose.pose.position.y;
        current_pos.z = 0.0;

        // 현재 목표점과의 거리 계산
        double distance = calculateDistance(current_pos, path_points_[current_goal_index_]);

        // 로봇의 속도 확인
        double linear_velocity = msg->twist.twist.linear.x;
        double angular_velocity = msg->twist.twist.angular.z;

        // 목표점 도달 여부 확인 (거리와 속도 조건)
        if (distance <= GOAL_THRESHOLD && 
            std::abs(linear_velocity) < 0.01 && 
            std::abs(angular_velocity) < 0.01) {
            current_goal_index_++;
            
            // 모든 목표점을 방문했으면 처음으로 돌아가기
            if (current_goal_index_ >= path_points_.size()) {
                RCLCPP_INFO(get_logger(), "Reached end of path. Restarting from beginning.");
                current_goal_index_ = 0;
            }
            
            // 새로운 목표점 발행
            goal_pub_->publish(path_points_[current_goal_index_]);
            RCLCPP_INFO(get_logger(), 
                "Published new goal point %zu/%zu: (%.2f, %.2f) meters", 
                current_goal_index_ + 1, path_points_.size(), 
                path_points_[current_goal_index_].x, 
                path_points_[current_goal_index_].y);
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr goal_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr init_timer_;
    std::vector<geometry_msgs::msg::Point> path_points_;
    size_t current_goal_index_ = 0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GoalPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 