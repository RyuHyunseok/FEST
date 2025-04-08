/*
 * Manual Goal Publisher Node
 * 
 * 1. 기능:
 *    - 사용자 입력을 통한 수동 목표점 발행
 *    - 입력된 좌표의 유효성 검사
 * 
 * 2. 구독 토픽: 없음
 * 
 * 3. 발행 토픽:
 *    - /goal_point: 사용자가 입력한 목표점 정보
 * 
 * 4. 사용 파일: 없음
 * 
 * 5. 저장 파일: 없음
 * 
 * 6. 주요 함수:
 *    - input_loop(): 사용자 입력 처리
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

        // 초기 목표점 (0,5) 발행
        auto goal_msg = geometry_msgs::msg::Point();
        goal_msg.x = 0.0;
        goal_msg.y = 5.0;
        goal_msg.z = 0.0;
        goal_pub_->publish(goal_msg);
        
        RCLCPP_INFO(get_logger(), "Goal Publisher initialized.");
        RCLCPP_INFO(get_logger(), "Published initial goal point: (0.00, 5.00) meters");
    }

    ~GoalPublisher() {
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr goal_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GoalPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 