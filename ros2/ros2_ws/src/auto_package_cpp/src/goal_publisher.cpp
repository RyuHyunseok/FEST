/*
 * Hybrid Goal Publisher Node
 * 
 * 기능:
 * - 자동/수동 모드 전환 가능한 목표점 발행
 * - 파일에서 읽은 경로의 목표점 자동 발행
 * - 사용자 입력을 통한 수동 목표점 발행
 * - 목표점 도달 감지 및 다음 목표점 발행
 * 
 * 토픽:
 * - 구독:
 *   - /odom: 로봇의 위치 정보
 * - 발행:
 *   - /goal_point: 목표점 정보
 */

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <iostream>
#include <string>
#include <thread>
#include <fstream>
#include <vector>
#include <cmath>
#include "auto_package_cpp/file_path.hpp"

// 전역 변수 선언 및 초기화
std::string PATH_FILE = auto_package_cpp::create_file_path("auto_package_cpp", "path/hoom2_path.txt");
const double GOAL_THRESHOLD = 0.35; // 목표점 도달 판단 거리 (미터)

// 파일 경로를 상수로 정의
// const std::string PATH_FILE = R"(C:\Users\SSAFY\Desktop\S12P21D106\ros2\ros2_ws\src\auto_package_cpp\path\hoom2_path.txt)";

class GoalPublisher : public rclcpp::Node {
public:
    GoalPublisher() : Node("goal_publisher"), 
                      current_path_index_(0), 
                      auto_mode_(false) {
        // 목표점 발행자 생성
        goal_pub_ = create_publisher<geometry_msgs::msg::Point>("/goal_point", 10);

        // 로봇 위치 구독자 생성
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10,
            std::bind(&GoalPublisher::odom_callback, this, std::placeholders::_1));

        // 자동 경로 출판 타이머 (1초마다 확인)
        auto_publish_timer_ = create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&GoalPublisher::check_publish_next_goal, this));

        // 입력을 받는 스레드 생성
        input_thread_ = std::thread([this]() { input_loop(); });

        RCLCPP_INFO(get_logger(), "Goal Publisher initialized.");
        RCLCPP_INFO(get_logger(), "Enter goal positions as 'x y' coordinates (-10 to 10 meters)");
        RCLCPP_INFO(get_logger(), "Map center is at (0, 0), with size 20x20 meters");
        RCLCPP_INFO(get_logger(), "Special commands:");
        RCLCPP_INFO(get_logger(), "  'q' - Quit the program");
        RCLCPP_INFO(get_logger(), "  'auto' - Toggle automatic path following from file");
        RCLCPP_INFO(get_logger(), "  'next' - Publish next goal from the path file");
        RCLCPP_INFO(get_logger(), "  'reset' - Reset path index to beginning");
        RCLCPP_INFO(get_logger(), "Using path file: %s", PATH_FILE.c_str());
    }

    ~GoalPublisher() {
        if (input_thread_.joinable()) {
            input_thread_.join();
        }
    }

private:
    bool load_path_from_file(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            RCLCPP_ERROR(get_logger(), "Failed to open path file: %s", filename.c_str());
            return false;
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
        
        file.close();
        
        if (path_points_.empty()) {
            RCLCPP_ERROR(get_logger(), "No valid points found in path file");
            return false;
        }
        
        RCLCPP_INFO(get_logger(), "Loaded %zu points from path file", path_points_.size());
        current_path_index_ = 0;
        return true;
    }

    double calculateDistance(const geometry_msgs::msg::Point& point1, const geometry_msgs::msg::Point& point2) {
        double dx = point1.x - point2.x;
        double dy = point1.y - point2.y;
        return std::sqrt(dx*dx + dy*dy);
    }

    void publish_next_goal() {
        if (path_points_.empty()) {
            RCLCPP_WARN(get_logger(), "No path points available. Loading path file...");
            if (!load_path_from_file(PATH_FILE)) {
                RCLCPP_ERROR(get_logger(), "Failed to load path file");
                return;
            }
        }
        
        if (current_path_index_ >= path_points_.size()) {
            RCLCPP_INFO(get_logger(), "Reached end of path, looping back to beginning");
            current_path_index_ = 0;
        }
        
        auto& goal = path_points_[current_path_index_];
        goal_pub_->publish(goal);
        
        RCLCPP_INFO(get_logger(), 
            "Published goal point %zu/%zu: (%.2f, %.2f) meters", 
            current_path_index_ + 1, path_points_.size(), goal.x, goal.y);
        
        current_path_index_++;
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        if (!auto_mode_ || path_points_.empty()) {
            return;
        }

        // 현재 로봇 위치
        geometry_msgs::msg::Point current_pos;
        current_pos.x = msg->pose.pose.position.x;
        current_pos.y = msg->pose.pose.position.y;
        current_pos.z = 0.0;

        // 현재 목표점과의 거리 계산
        double distance = calculateDistance(current_pos, path_points_[current_path_index_ - 1]);

        // 로봇의 속도 확인
        double linear_velocity = msg->twist.twist.linear.x;
        double angular_velocity = msg->twist.twist.angular.z;

        // 목표점 도달 여부 확인 (거리와 속도 조건)
        if (distance <= GOAL_THRESHOLD && 
            std::abs(linear_velocity) < 0.01 && 
            std::abs(angular_velocity) < 0.01) {
            RCLCPP_INFO(get_logger(), "Robot has arrived at destination");
            publish_next_goal();
        }
    }

    void check_publish_next_goal() {
        // 타이머는 유지하되, odom_callback에서 처리하도록 변경
    }

    void input_loop() {
        while (rclcpp::ok()) {
            std::cout << "\nEnter new goal position (x y) or 'q' to quit: ";
            std::string input;
            std::getline(std::cin, input);

            // 특수 명령어 처리
            if (input == "q" || input == "Q") {
                rclcpp::shutdown();
                break;
            }
            
            if (input == "auto") {
                if (!auto_mode_) {
                    // 수동 모드에서 자동 모드로 전환
                    if (path_points_.empty()) {
                        load_path_from_file(PATH_FILE);
                    }
                    auto_mode_ = true;
                    RCLCPP_INFO(get_logger(), "Auto mode enabled - will follow path automatically");
                    publish_next_goal();
                } else {
                    // 자동 모드에서 수동 모드로 전환
                    auto_mode_ = false;
                    RCLCPP_INFO(get_logger(), "Auto mode disabled - manual input mode");
                }
                continue;
            }
            
            if (input == "next") {
                if (path_points_.empty()) {
                    load_path_from_file(PATH_FILE);
                }
                publish_next_goal();
                continue;
            }
            
            if (input == "reset") {
                current_path_index_ = 0;
                RCLCPP_INFO(get_logger(), "Path index reset to beginning");
                continue;
            }

            // 좌표 입력 처리
            try {
                size_t space_pos = input.find(' ');
                if (space_pos == std::string::npos) {
                    RCLCPP_ERROR(get_logger(), "Invalid input format. Please enter two numbers separated by space.");
                    continue;
                }

                double x = std::stod(input.substr(0, space_pos));
                double y = std::stod(input.substr(space_pos + 1));

                if (x < -100.0 || x > 100.0 || y < -100.0 || y > 100.0) {
                    RCLCPP_ERROR(get_logger(), 
                        "Coordinates out of range. Please enter values between -100 and 100 meters.");
                    continue;
                }

                auto goal_msg = geometry_msgs::msg::Point();
                goal_msg.x = x;
                goal_msg.y = y;
                goal_msg.z = 0.0;

                goal_pub_->publish(goal_msg);
                RCLCPP_INFO(get_logger(), 
                    "Published new goal point: (%.2f, %.2f) meters", x, y);
            }
            catch (const std::exception& e) {
                RCLCPP_ERROR(get_logger(), "Invalid input format. Please enter two numbers separated by space.");
            }
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr goal_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr auto_publish_timer_;
    std::thread input_thread_;
    
    std::vector<geometry_msgs::msg::Point> path_points_;
    size_t current_path_index_;
    bool auto_mode_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GoalPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 