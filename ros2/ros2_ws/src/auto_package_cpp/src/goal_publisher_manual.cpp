/*
 * Manual Goal Publisher Node
 * 
 * 기능:
 * - 사용자 입력을 통해 목표점을 수동으로 발행
 * - 입력된 좌표의 유효성 검사
 * - 종료 명령 처리
 * 
 * 토픽:
 * - 발행:
 *   - /goal_point: 사용자가 입력한 목표점 정보
 */

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <iostream>
#include <string>
#include <thread>

class GoalPublisher : public rclcpp::Node {
public:
    GoalPublisher() : Node("goal_publisher") {
        // 목표점 발행자 생성
        goal_pub_ = create_publisher<geometry_msgs::msg::Point>("/goal_point", 10);

        // 입력을 받는 스레드 생성
        input_thread_ = std::thread([this]() { input_loop(); });

        RCLCPP_INFO(get_logger(), "Goal Publisher initialized.");
        RCLCPP_INFO(get_logger(), "Enter goal positions as 'x y' coordinates (-10 to 10 meters)");
        RCLCPP_INFO(get_logger(), "Map center is at (0, 0), with size 20x20 meters");
        RCLCPP_INFO(get_logger(), "You can enter new goals at any time.");
    }

    ~GoalPublisher() {
        if (input_thread_.joinable()) {
            input_thread_.join();
        }
    }

private:
    void input_loop() {
        while (rclcpp::ok()) {
            std::cout << "\nEnter new goal position (x y) or 'q' to quit: ";
            std::string input;
            std::getline(std::cin, input);

            if (input == "q" || input == "Q") {
                rclcpp::shutdown();
                break;
            }

            try {
                // 입력 문자열을 파싱하여 x, y 좌표 추출
                size_t space_pos = input.find(' ');
                if (space_pos == std::string::npos) {
                    RCLCPP_ERROR(get_logger(), "Invalid input format. Please enter two numbers separated by space.");
                    continue;
                }

                double x = std::stod(input.substr(0, space_pos));
                double y = std::stod(input.substr(space_pos + 1));

                // 입력 범위 검사 (-10 ~ 10 미터)
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
    std::thread input_thread_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GoalPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 