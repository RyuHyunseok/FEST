#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/int32.hpp"
#include <iostream>
#include <string>
#include <thread>
#include <fstream>
#include <vector>

// 파일 경로를 상수로 정의
const std::string PATH_FILE = R"(C:\Users\SSAFY\Desktop\S12P21D106\ros2\ros2_ws\src\auto_package_cpp\path\hoom2_path.txt)";

enum RobotState {
    DRIVING = 0,   // 주행 중
    STOPPED = 1,   // 정지 상태
    ARRIVED = 2,   // 목적지 도착
    INITIALIZING = 3 // 초기화 상태
};

class GoalPublisher : public rclcpp::Node {
public:
    GoalPublisher() : Node("goal_publisher"), 
                      current_path_index_(0), 
                      auto_mode_(false),
                      current_robot_state_(INITIALIZING),
                      previous_robot_state_(INITIALIZING),
                      arrived_handled_(false) {
        // 목표점 발행자 생성
        goal_pub_ = create_publisher<geometry_msgs::msg::Point>("/goal_point", 10);

        // 로봇 상태 구독자 생성
        robot_state_sub_ = create_subscription<std_msgs::msg::Int32>(
            "robot_state", 10,
            std::bind(&GoalPublisher::robot_state_callback, this, std::placeholders::_1));

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

    void robot_state_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        // 이전 상태를 저장
        previous_robot_state_ = current_robot_state_;
        
        // 현재 상태 업데이트
        current_robot_state_ = static_cast<RobotState>(msg->data);
        
        // 상태 변경 감지
        if (previous_robot_state_ != current_robot_state_) {
            // 상태가 변경되었을 때만 로그 출력
            if (auto_mode_) {
                switch (current_robot_state_) {
                    case ARRIVED:
                        RCLCPP_INFO(get_logger(), "Robot has arrived at destination");
                        arrived_handled_ = false; // 새로운 도착 상태 처리 준비
                        break;
                    case STOPPED:
                        RCLCPP_INFO(get_logger(), "Robot is stopped");
                        break;
                    case DRIVING:
                        RCLCPP_INFO(get_logger(), "Robot is now driving");
                        break;
                    case INITIALIZING:
                        RCLCPP_INFO(get_logger(), "Robot is initializing");
                        break;
                }
            }
            
            // DRIVING이나 다른 상태로 변경되면 도착 처리 플래그 초기화
            if (current_robot_state_ != ARRIVED) {
                arrived_handled_ = false;
            }
        }
    }

    void check_publish_next_goal() {
        if (auto_mode_ && current_robot_state_ == ARRIVED && !arrived_handled_) {
            RCLCPP_INFO(get_logger(), "Auto mode: Robot arrived, publishing next goal");
            publish_next_goal();
            arrived_handled_ = true; // 현재 도착 상태에 대한 처리 완료 표시
        }
    }

    void input_loop() {
        while (rclcpp::ok()) {
            std::cout << "\nEnter new goal position (x y) or 'q' to quit: ";
            std::string input;
            std::getline(std::cin, input);

            if (input == "q" || input == "Q") {
                rclcpp::shutdown();
                break;
            }
            
            if (input == "auto") {
                auto_mode_ = !auto_mode_;
                if (auto_mode_) {
                    if (path_points_.empty()) {
                        load_path_from_file(PATH_FILE);
                    }
                    RCLCPP_INFO(get_logger(), "Auto mode enabled - will follow path automatically");
                    publish_next_goal();
                } else {
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

            try {
                size_t space_pos = input.find(' ');
                if (space_pos == std::string::npos) {
                    RCLCPP_ERROR(get_logger(), "Invalid input format. Please enter two numbers separated by space.");
                    continue;
                }

                double x = std::stod(input.substr(0, space_pos));
                double y = std::stod(input.substr(space_pos + 1));

                if (x < -10.0 || x > 10.0 || y < -10.0 || y > 10.0) {
                    RCLCPP_ERROR(get_logger(), 
                        "Coordinates out of range. Please enter values between -10 and 10 meters.");
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
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr robot_state_sub_;
    rclcpp::TimerBase::SharedPtr auto_publish_timer_;
    std::thread input_thread_;
    
    std::vector<geometry_msgs::msg::Point> path_points_;
    size_t current_path_index_;
    bool auto_mode_;
    RobotState current_robot_state_;
    RobotState previous_robot_state_; // 이전 상태를 저장하기 위한 변수
    bool arrived_handled_; // 도착 상태 처리 여부를 추적하는 플래그
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GoalPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 