/*
 * Lattice Path Planner Node
 * 
 * 기능:
 * - 격자 기반의 후보 경로 생성
 * - 전역 경로를 기반으로 여러 후보 경로 생성
 * - 장애물 회피를 위한 최적 경로 선택
 * 
 * 토픽:
 * - 구독:
 *   - /global_path: 전역 경로
 *   - /odom: 로봇의 위치 정보
 *   - /scan: 라이다 스캔 데이터
 * - 발행:
 *   - /local_path: 선택된 로컬 경로
 *   - /candidate_paths: 생성된 후보 경로들
 */

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <cmath>
#include <vector>

using namespace std::chrono_literals;

class LatticePlanner : public rclcpp::Node {
public:
    LatticePlanner() : Node("lattice_planner"), 
                       is_odom_(false), 
                       is_global_path_(false),
                       is_lidar_(false),
                       local_path_size_(15),
                       robot_radius_(0.3),
                       num_paths_(7),  // 생성할 후보 경로 수
                       max_offset_(0.5) {  // 최대 측면 간격 (m)
        // Publishers
        local_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("local_path", 10);
        candidate_paths_pub_ = this->create_publisher<nav_msgs::msg::Path>("candidate_paths", 10);
        
        // Subscribers
        global_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "global_path", 10, 
            std::bind(&LatticePlanner::global_path_callback, this, std::placeholders::_1));
        
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, 
            std::bind(&LatticePlanner::odom_callback, this, std::placeholders::_1));

        lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, 
            std::bind(&LatticePlanner::lidar_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(50ms, std::bind(&LatticePlanner::timer_callback, this));

        // 후보 경로의 측면 간격 초기화 (균등 분포)
        lateral_steps_.clear();
        double step = (2.0 * max_offset_) / (num_paths_ - 1);
        for (int i = 0; i < num_paths_; i++) {
            lateral_steps_.push_back(-max_offset_ + i * step);
        }
    }

private:
    void timer_callback() {
        if (is_odom_ && is_global_path_ && is_lidar_) {
            generate_local_path();
        }
    }

    void generate_local_path() {
        size_t closest_idx = find_closest_global_path_point();
        
        // 후보 경로들 생성
        std::vector<nav_msgs::msg::Path> candidate_paths;
        nav_msgs::msg::Path all_candidates;  // 시각화를 위한 모든 후보 경로
        all_candidates.header.frame_id = "map";
        all_candidates.header.stamp = this->now();

        static double current_offset = 0.0;  // 현재 사용 중인 오프셋 저장

        // 단일 경로만 생성 (중앙 경로)
        nav_msgs::msg::Path single_path;
        single_path.header.frame_id = "map";
        single_path.header.stamp = this->now();
        
        for (size_t i = 0; i < local_path_size_; ++i) {
            size_t current_idx = closest_idx + i;
            if (current_idx >= global_path_msg_.poses.size()) break;

            geometry_msgs::msg::PoseStamped pose = global_path_msg_.poses[current_idx];
            // 오프셋 없이 원래 경로 사용
            single_path.poses.push_back(pose);
            all_candidates.poses.push_back(pose);
        }
        
        candidate_paths.push_back(single_path);
        current_local_path_ = single_path;

        /*
        // 각 오프셋에 대해 후보 경로 생성 (주석 처리된 원래 코드)
        for (double offset : lateral_steps_) {
            nav_msgs::msg::Path candidate_path;
            candidate_path.header.frame_id = "map";
            candidate_path.header.stamp = this->now();

            for (size_t i = 0; i < local_path_size_; ++i) {
                size_t current_idx = closest_idx + i;
                if (current_idx >= global_path_msg_.poses.size()) break;

                geometry_msgs::msg::PoseStamped pose = global_path_msg_.poses[current_idx];
                
                // 경로의 방향 계산
                double path_angle = 0.0;
                if (current_idx > 0) {
                    double dx = pose.pose.position.x - 
                              global_path_msg_.poses[current_idx-1].pose.position.x;
                    double dy = pose.pose.position.y - 
                              global_path_msg_.poses[current_idx-1].pose.position.y;
                    path_angle = std::atan2(dy, dx);
                }

                // 오프셋 적용
                pose.pose.position.x -= offset * std::sin(path_angle);
                pose.pose.position.y += offset * std::cos(path_angle);

                candidate_path.poses.push_back(pose);
                all_candidates.poses.push_back(pose);
            }
            candidate_paths.push_back(candidate_path);
        }

        // 현재 선택된 경로가 있고, 그 경로에 장애물이 없다면 현재 경로 유지
        if (!current_local_path_.poses.empty()) {
            bool obstacle_on_current = check_obstacles_on_path(current_local_path_);
            
            if (!obstacle_on_current) {
                // 현재 오프셋을 유지하면서 경로 업데이트
                nav_msgs::msg::Path updated_current_path;
                updated_current_path.header.frame_id = "map";
                updated_current_path.header.stamp = this->now();
                
                for (size_t i = 0; i < local_path_size_; ++i) {
                    size_t current_idx = closest_idx + i;
                    if (current_idx >= global_path_msg_.poses.size()) break;

                    geometry_msgs::msg::PoseStamped pose = global_path_msg_.poses[current_idx];
                    
                    // 경로의 방향 계산
                    double path_angle = 0.0;
                    if (current_idx > 0) {
                        double dx = pose.pose.position.x - 
                                  global_path_msg_.poses[current_idx-1].pose.position.x;
                        double dy = pose.pose.position.y - 
                                  global_path_msg_.poses[current_idx-1].pose.position.y;
                        path_angle = std::atan2(dy, dx);
                    }

                    // 현재 사용 중인 오프셋 적용
                    pose.pose.position.x -= current_offset * std::sin(path_angle);
                    pose.pose.position.y += current_offset * std::cos(path_angle);

                    updated_current_path.poses.push_back(pose);
                }
                
                current_local_path_ = updated_current_path;
                RCLCPP_DEBUG(this->get_logger(), "Keeping current path with offset: %.2f", current_offset);
            } else {
                // 장애물이 감지된 경우에만 새로운 경로 선택
                RCLCPP_INFO(this->get_logger(), "Obstacle detected on current path - selecting new path");
                select_best_path(candidate_paths);
                // 새로 선택된 경로의 오프셋 저장
                for (size_t i = 0; i < lateral_steps_.size(); ++i) {
                    if (candidate_paths[i].poses == current_local_path_.poses) {
                        current_offset = lateral_steps_[i];
                        break;
                    }
                }
            }
        } else {
            // 처음 시작할 때는 새로운 경로 선택
            RCLCPP_INFO(this->get_logger(), "Initial path selection");
            select_best_path(candidate_paths);
            // 초기 경로의 오프셋 저장
            for (size_t i = 0; i < lateral_steps_.size(); ++i) {
                if (candidate_paths[i].poses == current_local_path_.poses) {
                    current_offset = lateral_steps_[i];
                    break;
                }
            }
        }
        */

        // 후보 경로들 발행 (시각화용)
        candidate_paths_pub_->publish(all_candidates);
        
        // 선택된 지역 경로 발행
        if (!current_local_path_.poses.empty()) {
            local_path_pub_->publish(current_local_path_);
        }
    }

    void select_best_path(const std::vector<nav_msgs::msg::Path>& candidate_paths) {
        double min_cost = std::numeric_limits<double>::max();
        int best_idx = -1;
        static double prev_offset = 0.0;  // 이전에 선택된 경로의 오프셋을 저장

        // 각 후보 경로에 대해 평가
        for (size_t i = 0; i < candidate_paths.size(); ++i) {
            // 장애물이 있는 경로는 제외
            if (check_obstacles_on_path(candidate_paths[i])) {
                RCLCPP_DEBUG(this->get_logger(), "Path %zu has obstacles", i);
                continue;
            }

            // 비용 계산
            double offset_cost = std::abs(lateral_steps_[i]);  // 중앙 경로와의 거리
            double transition_cost = std::abs(lateral_steps_[i] - prev_offset);  // 이전 경로와의 차이
            
            // 전체 비용 계산 (가중치 적용)
            double total_cost = offset_cost + 0.5 * transition_cost;
            
            if (total_cost < min_cost) {
                min_cost = total_cost;
                best_idx = i;
            }
        }

        // 유효한 경로를 찾은 경우
        if (best_idx >= 0) {
            current_local_path_ = candidate_paths[best_idx];
            prev_offset = lateral_steps_[best_idx];  // 선택된 경로의 오프셋 저장
            
            RCLCPP_INFO(this->get_logger(), 
                "Selected new path - offset: %.2f, cost: %.2f", 
                lateral_steps_[best_idx], 
                min_cost);
        } else {
            // 유효한 경로를 찾지 못한 경우, 현재 경로 유지
            RCLCPP_WARN(this->get_logger(), "No valid path found - keeping current path");
        }
    }

    bool check_obstacles_on_path(const nav_msgs::msg::Path& path) {
        double robot_yaw = get_robot_yaw();
        double robot_x = odom_msg_.pose.pose.position.x;
        double robot_y = odom_msg_.pose.pose.position.y;

        for (const auto& path_pose : path.poses) {
            double path_x = path_pose.pose.position.x;
            double path_y = path_pose.pose.position.y;

            for (size_t i = 0; i < lidar_msg_.ranges.size(); ++i) {
                double range = lidar_msg_.ranges[i];
                
                if (!std::isfinite(range) || range < robot_radius_) continue;

                double angle = lidar_msg_.angle_min + i * lidar_msg_.angle_increment;
                
                // 라이다 포인트를 전역 좌표로 변환
                double obstacle_x = robot_x + range * std::cos(angle + robot_yaw);
                double obstacle_y = robot_y + range * std::sin(angle + robot_yaw);

                // 장애물과 경로 포인트 사이의 거리 계산
                double dx = path_x - obstacle_x;
                double dy = path_y - obstacle_y;
                double distance = std::sqrt(dx*dx + dy*dy);

                if (distance < robot_radius_ * 1.5) {
                    RCLCPP_INFO(this->get_logger(), 
                        "Obstacle detected near path at (%.2f, %.2f), distance: %.2f",
                        obstacle_x, obstacle_y, distance);
                    return true;
                }
            }
        }
        return false;
    }

    size_t find_closest_global_path_point() {
        double min_dist = std::numeric_limits<double>::max();
        size_t closest_idx = 0;
        double robot_x = odom_msg_.pose.pose.position.x;
        double robot_y = odom_msg_.pose.pose.position.y;

        for (size_t i = 0; i < global_path_msg_.poses.size(); ++i) {
            double dx = global_path_msg_.poses[i].pose.position.x - robot_x;
            double dy = global_path_msg_.poses[i].pose.position.y - robot_y;
            double dist = std::sqrt(dx*dx + dy*dy);
            
            if (dist < min_dist) {
                min_dist = dist;
                closest_idx = i;
            }
        }
        return closest_idx;
    }

    void global_path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
        global_path_msg_ = *msg;
        is_global_path_ = true;
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        odom_msg_ = *msg;
        is_odom_ = true;
    }

    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        lidar_msg_ = *msg;
        is_lidar_ = true;
    }

    double get_robot_yaw() {
        tf2::Quaternion q(
            odom_msg_.pose.pose.orientation.x,
            odom_msg_.pose.pose.orientation.y,
            odom_msg_.pose.pose.orientation.z,
            odom_msg_.pose.pose.orientation.w);
        
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        return yaw;
    }

    // Publishers
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr candidate_paths_pub_;
    
    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Messages
    nav_msgs::msg::Path global_path_msg_;
    nav_msgs::msg::Path current_local_path_;
    nav_msgs::msg::Odometry odom_msg_;
    sensor_msgs::msg::LaserScan lidar_msg_;
    
    // State flags
    bool is_odom_;
    bool is_global_path_;
    bool is_lidar_;
    
    // Parameters
    int local_path_size_;
    double robot_radius_;
    int num_paths_;
    double max_offset_;
    std::vector<double> lateral_steps_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LatticePlanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
