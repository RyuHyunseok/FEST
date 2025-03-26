#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <cmath>
#include <vector>
#include <algorithm>

const double M_PI = std::acos(-1);

using namespace std::chrono_literals;

class LocalCostMapGenerator : public rclcpp::Node 
{
public:
    LocalCostMapGenerator() : Node("local_cost_map_generator"),
                            has_odom_(false),
                            has_lidar_(false),
                            resolution_(0.05),      // 5cm 해상도
                            width_(200),            // 10m width (200 * 0.05m)
                            height_(200),           // 10m height (200 * 0.05m)
                            inflation_radius_(0.3), // 30cm 장애물 팽창 반경
                            cost_scaling_factor_(2.0) // 장애물로부터의 거리에 따른 비용 감소 계수
    {
        // 현재 맵 상태를 저장할 OccupancyGrid 메시지 초기화
        cost_map_.header.frame_id = "map";
        cost_map_.info.resolution = resolution_;
        cost_map_.info.width = width_;
        cost_map_.info.height = height_;
        cost_map_.info.origin.position.z = 0.0;
        cost_map_.info.origin.orientation.w = 1.0;
        cost_map_.data.resize(width_ * height_, 0);

        // Publisher
        cost_map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/local_cost_map", 10);

        // Subscribers
        lidar_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, 
            std::bind(&LocalCostMapGenerator::lidar_callback, this, std::placeholders::_1));

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, 
            std::bind(&LocalCostMapGenerator::odom_callback, this, std::placeholders::_1));

        // Timer for periodic updates
        update_timer_ = create_wall_timer(
            100ms, std::bind(&LocalCostMapGenerator::update_cost_map, this));
    }

private:
    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        lidar_data_ = *msg;
        has_lidar_ = true;
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        odom_data_ = *msg;
        has_odom_ = true;
        
        // Extract robot's yaw
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        robot_yaw_ = yaw;
    }

    void update_cost_map()
    {
        if (!has_odom_ || !has_lidar_) {
            return;
        }

        // 로봇 위치를 중심으로 맵의 원점 설정
        double robot_x = odom_data_.pose.pose.position.x;
        double robot_y = odom_data_.pose.pose.position.y;
        
        cost_map_.info.origin.position.x = robot_x - (width_ * resolution_ / 2.0);
        cost_map_.info.origin.position.y = robot_y - (height_ * resolution_ / 2.0);
        
        // 맵 초기화 (모두 0으로)
        std::fill(cost_map_.data.begin(), cost_map_.data.end(), 0);
        
        // 라이다 데이터 처리하여 장애물 위치 기록
        process_lidar_data();
        
        // 장애물 주변에 비용 확장 (팽창)
        inflate_obstacles();
        
        // 로봇 자신 위치의 비용 제거 (로봇 영역에 해당하는 셀 비우기)
        clear_robot_footprint();
        
        // 맵 발행
        cost_map_.header.stamp = now();
        cost_map_pub_->publish(cost_map_);
    }

    void process_lidar_data()
    {
        double robot_x = odom_data_.pose.pose.position.x;
        double robot_y = odom_data_.pose.pose.position.y;
        
        double angle = lidar_data_.angle_min;
        
        for (const auto& range : lidar_data_.ranges) {
            // 유효한 범위의 데이터만 처리
            if (std::isfinite(range) && range > lidar_data_.range_min && range < lidar_data_.range_max) {
                // 라이다 좌표계에서 전역 좌표계로 변환
                double obstacle_x = robot_x + range * std::cos(angle + robot_yaw_);
                double obstacle_y = robot_y + range * std::sin(angle + robot_yaw_);
                
                // 전역 좌표를 맵 좌표(그리드)로 변환
                int grid_x = static_cast<int>((obstacle_x - cost_map_.info.origin.position.x) / resolution_);
                int grid_y = static_cast<int>((obstacle_y - cost_map_.info.origin.position.y) / resolution_);
                
                // 맵 범위 내인지 확인
                if (grid_x >= 0 && grid_x < width_ && grid_y >= 0 && grid_y < height_) {
                    // 장애물 위치에 100(점유) 표시
                    cost_map_.data[grid_y * width_ + grid_x] = 100;
                }
            }
            
            angle += lidar_data_.angle_increment;
        }
    }

    void inflate_obstacles()
    {
        // 임시 맵 생성 (팽창 처리를 위해)
        std::vector<int8_t> inflated_map = cost_map_.data;
        
        // 장애물 팽창 반경(셀 단위)
        int inflation_radius_cells = static_cast<int>(inflation_radius_ / resolution_);
        
        for (int y = 0; y < height_; ++y) {
            for (int x = 0; x < width_; ++x) {
                int index = y * width_ + x;
                
                // 장애물인 경우 주변에 비용 할당
                if (cost_map_.data[index] == 100) {
                    for (int dy = -inflation_radius_cells; dy <= inflation_radius_cells; ++dy) {
                        for (int dx = -inflation_radius_cells; dx <= inflation_radius_cells; ++dx) {
                            int nx = x + dx;
                            int ny = y + dy;
                            
                            // 맵 범위 내인지 확인
                            if (nx >= 0 && nx < width_ && ny >= 0 && ny < height_) {
                                // 이미 장애물인 위치는 건너뜀
                                if (cost_map_.data[ny * width_ + nx] == 100) {
                                    continue;
                                }
                                
                                // 장애물로부터의 거리 계산
                                double distance = std::hypot(dx * resolution_, dy * resolution_);
                                
                                // 팽창 반경 내인 경우만 비용 할당
                                if (distance <= inflation_radius_) {
                                    // 거리에 따른 비용 계산 (거리가 멀수록 낮은 비용)
                                    // 90에서 1까지의 범위로 설정
                                    double factor = std::exp(-cost_scaling_factor_ * distance);
                                    int cost = static_cast<int>(90.0 * factor);
                                    
                                    // 기존 비용과 비교하여 높은 값으로 설정
                                    int cell_index = ny * width_ + nx;
                                    inflated_map[cell_index] = std::max(inflated_map[cell_index], static_cast<int8_t>(cost));
                                }
                            }
                        }
                    }
                }
            }
        }
        
        cost_map_.data = inflated_map;
    }

    void clear_robot_footprint()
    {
        // 로봇의 위치를 그리드 좌표로 변환
        double robot_x = odom_data_.pose.pose.position.x;
        double robot_y = odom_data_.pose.pose.position.y;
        
        int robot_grid_x = static_cast<int>((robot_x - cost_map_.info.origin.position.x) / resolution_);
        int robot_grid_y = static_cast<int>((robot_y - cost_map_.info.origin.position.y) / resolution_);
        
        // 로봇 반경(footprint) 설정 - 약 30cm로 설정
        double robot_footprint_radius = 0.3;
        int footprint_cells = static_cast<int>(robot_footprint_radius / resolution_);
        
        // 로봇 영역에 해당하는 셀 비우기
        for (int dy = -footprint_cells; dy <= footprint_cells; ++dy) {
            for (int dx = -footprint_cells; dx <= footprint_cells; ++dx) {
                int nx = robot_grid_x + dx;
                int ny = robot_grid_y + dy;
                
                // 맵 범위 내인지 확인
                if (nx >= 0 && nx < width_ && ny >= 0 && ny < height_) {
                    // 셀 중심까지의 거리 계산
                    double distance = std::hypot(dx * resolution_, dy * resolution_);
                    
                    // 로봇 반경 내에 있는 경우 비용 제거
                    if (distance <= robot_footprint_radius) {
                        cost_map_.data[ny * width_ + nx] = 0;
                    }
                }
            }
        }
    }

    // Publishers
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr cost_map_pub_;
    
    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr update_timer_;
    
    // Data
    nav_msgs::msg::OccupancyGrid cost_map_;
    sensor_msgs::msg::LaserScan lidar_data_;
    nav_msgs::msg::Odometry odom_data_;
    
    // Parameters
    double resolution_;  // 맵 해상도 (m/cell)
    int width_;          // 맵 너비 (cells)
    int height_;         // 맵 높이 (cells)
    double inflation_radius_;  // 장애물 팽창 반경 (m)
    double cost_scaling_factor_;  // 비용 감소 계수
    double robot_yaw_;   // 로봇의 방향
    
    // 상태 플래그
    bool has_odom_;
    bool has_lidar_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LocalCostMapGenerator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 