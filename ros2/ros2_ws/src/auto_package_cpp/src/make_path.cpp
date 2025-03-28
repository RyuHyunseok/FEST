/*
 * Path Recording Node
 * 
 * 기능:
 * - 로봇의 이동 경로를 실시간으로 기록
 * - 일정 거리 간격으로 웨이포인트 저장
 * - 기록된 경로를 파일로 저장
 * 
 * 토픽:
 * - 구독:
 *   - /odom: 로봇의 위치 정보
 * - 발행:
 *   - /global_path: 기록된 경로
 */

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include <fstream>
#include <string>
#include <cmath>
#include <memory>
#include "auto_package_cpp/file_path.hpp"

// 전역 변수 선언 및 초기화
std::string PATH_FILE = auto_package_cpp::create_file_path("auto_package_cpp", "path/test_save.txt");

// 파일 경로를 상수로 정의
// const std::string PATH_FILE = R"(C:\Users\SSAFY\Desktop\S12P21D106\ros2\ros2_ws\src\auto_package_cpp\path\test_save.txt)";

using namespace std::chrono_literals;

class MakePath : public rclcpp::Node
{
private:
    // 발행자
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    
    // 구독자
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    
    // 파일 스트림
    std::ofstream file_;
    
    // 상태 변수
    bool is_odom_;
    double prev_x_;
    double prev_y_;
    
    // 메시지
    nav_msgs::msg::Path path_msg_;

    void listener_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if (!is_odom_) {
            is_odom_ = true;
            prev_x_ = msg->pose.pose.position.x;
            prev_y_ = msg->pose.pose.position.y;
        } else {
            geometry_msgs::msg::PoseStamped waypoint_pose;
            double x = msg->pose.pose.position.x;
            double y = msg->pose.pose.position.y;
            double distance = std::sqrt(std::pow(x - prev_x_, 2) + std::pow(y - prev_y_, 2));
            
            if (distance > 0.1) {
                waypoint_pose.pose.position.x = x;
                waypoint_pose.pose.position.y = y;
                waypoint_pose.pose.orientation.w = 1.0;
                
                path_msg_.poses.push_back(waypoint_pose);
                path_pub_->publish(path_msg_);
                
                // 파일에 데이터 쓰기
                file_ << x << "\t" << y << std::endl;
                
                prev_x_ = x;
                prev_y_ = y;
            }
        }
    }

public:
    MakePath() : Node("make_path"), is_odom_(false), prev_x_(0.0), prev_y_(0.0)
    {
        // 발행자 생성
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("global_path", 10);
        
        // 구독자 생성
        subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&MakePath::listener_callback, this, std::placeholders::_1));
        
        // 파일 열기 (경로 상수 사용)
        file_.open(PATH_FILE, std::ios::out);
        
        // Path 메시지 초기화
        path_msg_.header.frame_id = "map";
    }
    
    ~MakePath()
    {
        // 파일 닫기
        if (file_.is_open()) {
            file_.close();
        }
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MakePath>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 