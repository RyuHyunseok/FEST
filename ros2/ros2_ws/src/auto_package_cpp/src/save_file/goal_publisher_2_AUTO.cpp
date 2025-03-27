#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/int32.hpp"
#include <fstream>
#include <vector>
#include "auto_package_cpp/file_path.hpp"

// 전역 변수 선언 및 초기화
std::string PATH_FILE = auto_package_cpp::create_file_path("auto_package_cpp", "path/hoom2_path.txt");

enum RobotState {
    DRIVING = 0,   // 주행 중
    STOPPED = 1,   // 정지 상태
    ARRIVED = 2,   // 목적지 도착
    INITIALIZING = 3 // 초기화 상태
};

class AutoGoalPublisher : public rclcpp::Node {
public:
    AutoGoalPublisher() : Node("auto_goal_publisher"), 
                         current_path_index_(0), 
                         current_robot_state_(INITIALIZING),
                         previous_robot_state_(INITIALIZING),
                         arrived_handled_(false) {
        // 목표점 발행자 생성 (QoS 설정으로 latching 활성화)
        rclcpp::QoS qos(10);
        qos.transient_local();  // 이것이 latching과 동일한 효과
        goal_pub_ = create_publisher<geometry_msgs::msg::Point>("/goal_point", qos);

        // 로봇 상태 구독자 생성
        robot_state_sub_ = create_subscription<std_msgs::msg::Int32>(
            "robot_state", 10,
            std::bind(&AutoGoalPublisher::robot_state_callback, this, std::placeholders::_1));

        // 자동 경로 출판 타이머 (1초마다 확인)
        auto_publish_timer_ = create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&AutoGoalPublisher::check_publish_next_goal, this));

        RCLCPP_INFO(get_logger(), "Auto Goal Publisher initialized.");
        RCLCPP_INFO(get_logger(), "Using path file: %s", PATH_FILE.c_str());
        
        // 시작 시 경로 파일 로드
        if (load_path_from_file(PATH_FILE)) {
            publish_next_goal();
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
        goal_pub_->publish(goal);  // latching publisher이므로 한 번만 발행해도 됨
        
        RCLCPP_INFO(get_logger(), 
            "Published goal point %zu/%zu: (%.2f, %.2f) meters", 
            current_path_index_ + 1, path_points_.size(), goal.x, goal.y);
        
        current_path_index_++;
    }

    void robot_state_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        previous_robot_state_ = current_robot_state_;
        current_robot_state_ = static_cast<RobotState>(msg->data);
        
        if (previous_robot_state_ != current_robot_state_) {
            switch (current_robot_state_) {
                case ARRIVED:
                    RCLCPP_INFO(get_logger(), "Robot has arrived at destination");
                    arrived_handled_ = false;
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
            
            if (current_robot_state_ != ARRIVED) {
                arrived_handled_ = false;
            }
        }
    }

    void check_publish_next_goal() {
        if (current_robot_state_ == ARRIVED && !arrived_handled_) {
            RCLCPP_INFO(get_logger(), "Robot arrived, publishing next goal");
            publish_next_goal();
            arrived_handled_ = true;
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr goal_pub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr robot_state_sub_;
    rclcpp::TimerBase::SharedPtr auto_publish_timer_;
    
    std::vector<geometry_msgs::msg::Point> path_points_;
    size_t current_path_index_;
    RobotState current_robot_state_;
    RobotState previous_robot_state_;
    bool arrived_handled_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AutoGoalPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 