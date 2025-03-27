#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <iostream>
#include <thread>
#include <chrono>

class TestOdomPublisher : public rclcpp::Node {
public:
    TestOdomPublisher() : Node("test_odom_publisher") {
        // odom 발행자 생성
        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        
        // 사용자 입력을 받는 스레드 시작
        input_thread_ = std::thread(&TestOdomPublisher::getUserInput, this);
        
        // odom 발행 타이머 생성 (1초 간격)
        timer_ = create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&TestOdomPublisher::publishOdom, this));
            
        RCLCPP_INFO(get_logger(), "Test Odom Publisher initialized.");
        RCLCPP_INFO(get_logger(), "Enter coordinates in format: x y");
    }

    ~TestOdomPublisher() {
        if (timer_) {
            timer_->cancel();
        }
        if (input_thread_.joinable()) {
            input_thread_.join();
        }
    }

private:
    void getUserInput() {
        while (rclcpp::ok()) {
            std::cout << "Enter x y coordinates (or 'q' to quit): ";
            std::string input;
            std::getline(std::cin, input);
            
            if (input == "q") {
                rclcpp::shutdown();
                break;
            }
            
            double x, y;
            if (std::istringstream(input) >> x >> y) {
                std::lock_guard<std::mutex> lock(mutex_);
                current_x_ = x;
                current_y_ = y;
                RCLCPP_INFO(get_logger(), "Updated position to: (%.2f, %.2f)", x, y);
            } else {
                RCLCPP_ERROR(get_logger(), "Invalid input. Please enter two numbers.");
            }
        }
    }

    void publishOdom() {
        nav_msgs::msg::Odometry odom_msg;
        
        // 메시지 헤더 설정
        odom_msg.header.stamp = this->now();
        odom_msg.header.frame_id = "map";
        odom_msg.child_frame_id = "base_link";
        
        // 위치 정보 설정
        std::lock_guard<std::mutex> lock(mutex_);
        odom_msg.pose.pose.position.x = current_x_;
        odom_msg.pose.pose.position.y = current_y_;
        odom_msg.pose.pose.position.z = 0.0;
        
        // 방향 정보 설정 (기본값)
        odom_msg.pose.pose.orientation.x = 0.0;
        odom_msg.pose.pose.orientation.y = 0.0;
        odom_msg.pose.pose.orientation.z = 0.0;
        odom_msg.pose.pose.orientation.w = 1.0;
        
        // 속도 정보 설정 (기본값)
        odom_msg.twist.twist.linear.x = 0.0;
        odom_msg.twist.twist.linear.y = 0.0;
        odom_msg.twist.twist.linear.z = 0.0;
        odom_msg.twist.twist.angular.x = 0.0;
        odom_msg.twist.twist.angular.y = 0.0;
        odom_msg.twist.twist.angular.z = 0.0;
        
        // odom 메시지 발행
        odom_pub_->publish(odom_msg);
    }

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::thread input_thread_;
    std::mutex mutex_;
    double current_x_ = 0.0;
    double current_y_ = 0.0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TestOdomPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 