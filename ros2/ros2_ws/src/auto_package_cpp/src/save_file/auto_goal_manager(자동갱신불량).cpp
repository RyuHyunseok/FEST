#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/int32.hpp"
#include <fstream>
#include <vector>
#include "auto_package_cpp/file_path.hpp"

// enum을 클래스 선언 전에 정의
enum RobotState {
    DRIVING = 0,   // 주행 중
    STOPPED = 1,   // 정지 상태
    ARRIVED = 2,   // 목적지 도착
    INITIALIZING = 3 // 초기화 상태
};

class AutoGoalManager : public rclcpp::Node {
public:
    AutoGoalManager() : Node("auto_goal_manager"), 
                       current_path_index_(0),
                       current_robot_state_(INITIALIZING),
                       previous_robot_state_(INITIALIZING),
                       arrived_handled_(false) {
        
        // 목표점 발행자 생성
        goal_pub_ = create_publisher<geometry_msgs::msg::Point>("/goal_point", 10);
        
        // 로봇 상태 구독자 생성
        robot_state_sub_ = create_subscription<std_msgs::msg::Int32>(
            "robot_state", 10,
            std::bind(&AutoGoalManager::robot_state_callback, this, std::placeholders::_1));

        // 경로 파일 로드
        std::string path_file = auto_package_cpp::create_file_path("auto_package_cpp", "path/hoom2_path.txt");
        
        // 파일 경로 로그 추가
        RCLCPP_INFO(get_logger(), "Attempting to load file: %s", path_file.c_str());
        
        if (!load_path_from_file(path_file)) {
            RCLCPP_ERROR(get_logger(), "Failed to load path file");
            return;  // 여기서 return하면 publish_next_goal()이 실행되지 않음
        }

        // 첫 번째 목표점 발행
        RCLCPP_INFO(get_logger(), "Attempting to publish first goal");
        publish_next_goal();
        
        // 타이머를 check_publish_next_goal로 변경
        timer_ = create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&AutoGoalManager::check_publish_next_goal, this));
        
        RCLCPP_INFO(get_logger(), "Auto Goal Manager initialized");
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
        return true;
    }

    void publish_next_goal() {
        if (path_points_.empty()) {
            RCLCPP_WARN(get_logger(), "No path points available");
            return;
        }
        
        if (current_path_index_ >= path_points_.size()) {
            RCLCPP_INFO(get_logger(), "Reached end of path, looping back to beginning");
            current_path_index_ = 0;
        }
        
        auto& goal = path_points_[current_path_index_];
        goal_pub_->publish(goal);
        
        RCLCPP_INFO(get_logger(), 
            "Published goal point %zu/%zu: (%.2f, %.2f)", 
            current_path_index_ + 1, path_points_.size(), goal.x, goal.y);
        
        // 인덱스 증가는 로봇이 목표점에 도착했을 때만 수행
    }

    void check_publish_next_goal() {
        if (current_robot_state_ == ARRIVED && !arrived_handled_) {
            RCLCPP_INFO(get_logger(), "Robot arrived, publishing next goal");
            current_path_index_++;
            publish_next_goal();
            arrived_handled_ = true;
        }
    }

    void robot_state_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        previous_robot_state_ = current_robot_state_;
        current_robot_state_ = static_cast<RobotState>(msg->data);
        
        if (current_robot_state_ != ARRIVED) {
            arrived_handled_ = false;
        }
    }

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr goal_pub_;
    
    // Subscribers
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr robot_state_sub_;
    
    // Path data
    std::vector<geometry_msgs::msg::Point> path_points_;
    size_t current_path_index_;

    rclcpp::TimerBase::SharedPtr timer_;

    // RobotState 변수들
    RobotState current_robot_state_;
    RobotState previous_robot_state_;
    bool arrived_handled_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AutoGoalManager>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
