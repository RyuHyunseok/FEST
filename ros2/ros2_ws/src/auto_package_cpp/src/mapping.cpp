/*
 * SLAM Mapping Node
 * 
 * 기능:
 * - 라이다 데이터를 기반으로 환경 맵 생성
 * - 로봇의 위치 정보를 이용한 맵 업데이트
 * - 생성된 맵을 PGM 파일로 저장
 * 
 * 토픽:
 * - 구독:
 *   - /scan: 라이다 스캔 데이터
 *   - /odom: 로봇의 위치 정보
 * - 발행:
 *   - /map: 생성된 맵 데이터
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/map_meta_data.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <Eigen/Dense>
#include <cmath>
#include <fstream>
#include <memory>
#include <vector>
#include <string>
#include "auto_package_cpp/file_path.hpp"

// 전역 변수 선언 및 초기화
std::string MAP_FILE = auto_package_cpp::create_file_path("auto_package_cpp", "path/map_save.pgm");

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

const double MAP_WIDTH = 20.0;  // 맵의 너비 (미터)
const double MAP_HEIGHT = 20.0; // 맵의 높이 (미터)
const double MAP_CENTER_X = 0.0;  // 맵의 중심 X 좌표 (미터)
const double MAP_CENTER_Y = 0.0;  // 맵의 중심 Y 좌표 (미터)

// 파일 경로를 상수로 정의
// const std::string MAP_FILE = R"(C:\Users\SSAFY\Desktop\S12P21D106\ros2\ros2_ws\src\auto_package_cpp\path\map.pgm)";

struct Point {
    int x;
    int y;
    Point(int _x, int _y) : x(_x), y(_y) {}
};

struct MapParams {
    double MAP_RESOLUTION;
    double OCCUPANCY_UP;
    double OCCUPANCY_DOWN;
    std::pair<double, double> MAP_CENTER;
    std::pair<double, double> MAP_SIZE;
    std::string MAP_FILENAME;
    double MAPVIS_RESIZE_SCALE;
};

class Mapping {
public:
    Mapping(const MapParams& params)
        : map_resolution_(params.MAP_RESOLUTION)
        , map_center_(params.MAP_CENTER)
        , occu_up_(params.OCCUPANCY_UP)
        , occu_down_(params.OCCUPANCY_DOWN)
        , map_filename_(params.MAP_FILENAME)
        , map_vis_resize_scale_(params.MAPVIS_RESIZE_SCALE)
    {
        map_size_ = std::make_pair(
            static_cast<int>(params.MAP_SIZE.first / map_resolution_),
            static_cast<int>(params.MAP_SIZE.second / map_resolution_)
        );
        
        map_data_.resize(map_size_.first * map_size_.second, 0.5f);
        
        // 레이저 스캐너의 좌표계를 로봇 좌표계로 변환하는 행렬
        T_r_l_ = Eigen::Matrix3d::Zero();
        T_r_l_(0, 0) = 1.0;  // x축은 그대로
        T_r_l_(1, 1) = 1.0;  // y축은 그대로
        T_r_l_(2, 2) = 1.0;  // z축은 그대로
    }

    void update(const Eigen::Vector3d& pose, const Eigen::MatrixXd& laser) {
        Eigen::Matrix3d pose_mat;
        pose_mat << std::cos(pose(2)), -std::sin(pose(2)), pose(0),
                    std::sin(pose(2)),  std::cos(pose(2)), pose(1),
                    0, 0, 1;
                    
        Eigen::Matrix3d transform = pose_mat * T_r_l_;
        
        Eigen::MatrixXd laser_homogeneous(3, laser.cols());
        laser_homogeneous.topRows(2) = laser;
        laser_homogeneous.row(2).setOnes();
        
        Eigen::MatrixXd laser_global = transform * laser_homogeneous;
        
        double pose_x = (pose(0) - map_center_.first + (map_size_.first * map_resolution_) / 2) / map_resolution_;
        double pose_y = (pose(1) - map_center_.second + (map_size_.second * map_resolution_) / 2) / map_resolution_;
        // 맵핑을 위한 좌표 변환 (맵의 중심점 이동 고려)
        // double pose_x = (pose(0) + (map_size_.first * map_resolution_) / 2) / map_resolution_;
        // double pose_y = (pose(1) + (map_size_.second * map_resolution_) / 2) / map_resolution_;
        
        for (int i = 0; i < laser_global.cols(); ++i) {
            double laser_x = (laser_global(0, i) - map_center_.first + (map_size_.first * map_resolution_) / 2) / map_resolution_;
            double laser_y = (laser_global(1, i) - map_center_.second + (map_size_.second * map_resolution_) / 2) / map_resolution_;
            // double laser_x = (laser_global(0, i) + (map_size_.first * map_resolution_) / 2) / map_resolution_;
            // double laser_y = (laser_global(1, i) + (map_size_.second * map_resolution_) / 2) / map_resolution_;
            
            Point p1(static_cast<int>(pose_x), static_cast<int>(pose_y));
            Point p2(static_cast<int>(laser_x), static_cast<int>(laser_y));
            
            auto points = createLineIterator(p1, p2);
            
            if (points.empty()) continue;
            
            for (size_t j = 0; j < points.size() - 1; ++j) {
                float& value = at(points[j].y, points[j].x);
                value -= occu_down_;
                value = std::min(1.0f, std::max(0.0f, value));
            }
            
            if (!points.empty()) {
                float& value = at(points.back().y, points.back().x);
                value += occu_up_;
                value = std::min(1.0f, std::max(0.0f, value));
            }
        }
    }

    void save_map() {
        std::ofstream file(map_filename_, std::ios::binary);
        if (!file) {
            RCLCPP_ERROR(rclcpp::get_logger("mapping"), "Failed to open file for writing: %s", map_filename_.c_str());
            return;
        }

        // PGM 형식으로 저장
        file << "P5\n" << map_size_.second << " " << map_size_.first << "\n255\n";
        
        std::vector<unsigned char> buffer(map_size_.first * map_size_.second);
        for (size_t i = 0; i < map_data_.size(); ++i) {
            buffer[i] = static_cast<unsigned char>(map_data_[i] * 255);
        }
        
        file.write(reinterpret_cast<char*>(buffer.data()), buffer.size());
    }

    const std::vector<float>& get_map_data() const { return map_data_; }
    std::pair<int, int> get_map_size() const { return map_size_; }
    const std::string& get_map_filename() const { return map_filename_; }

private:
    std::vector<Point> createLineIterator(const Point& p1, const Point& p2) {
        std::vector<Point> points;
        
        int dx = std::abs(p2.x - p1.x);
        int dy = std::abs(p2.y - p1.y);
        
        int x = p1.x;
        int y = p1.y;
        
        int step_x = (p2.x > p1.x) ? 1 : -1;
        int step_y = (p2.y > p1.y) ? 1 : -1;
        
        if (dx > dy) {
            int p = 2 * dy - dx;
            while (x != p2.x) {
                if (x >= 0 && x < map_size_.second && y >= 0 && y < map_size_.first) {
                    points.emplace_back(x, y);
                }
                if (p >= 0) {
                    y += step_y;
                    p -= 2 * dx;
                }
                x += step_x;
                p += 2 * dy;
            }
        } else {
            int p = 2 * dx - dy;
            while (y != p2.y) {
                if (x >= 0 && x < map_size_.second && y >= 0 && y < map_size_.first) {
                    points.emplace_back(x, y);
                }
                if (p >= 0) {
                    x += step_x;
                    p -= 2 * dy;
                }
                y += step_y;
                p += 2 * dx;
            }
        }
        
        if (x >= 0 && x < map_size_.second && y >= 0 && y < map_size_.first) {
            points.emplace_back(x, y);
        }
        
        return points;
    }

    float& at(int y, int x) { return map_data_[y * map_size_.second + x]; }
    const float& at(int y, int x) const { return map_data_[y * map_size_.second + x]; }

    std::vector<float> map_data_;
    double map_resolution_;
    std::pair<int, int> map_size_;
    std::pair<double, double> map_center_;
    double occu_up_;
    double occu_down_;
    std::string map_filename_;
    double map_vis_resize_scale_;
    Eigen::Matrix3d T_r_l_;
};

class MapperNode : public rclcpp::Node {
public:
    MapperNode() : Node("mapper"), is_map_create_(true), is_odom_(false) {
        MapParams params;
        params.MAP_RESOLUTION = 0.05;
        params.OCCUPANCY_UP = 0.1;
        params.OCCUPANCY_DOWN = 0.01;
        params.MAP_CENTER = {MAP_CENTER_X, MAP_CENTER_Y};  // 전역 변수 사용
        params.MAP_SIZE = {MAP_WIDTH, MAP_HEIGHT};
        params.MAP_FILENAME = MAP_FILE;
        params.MAPVIS_RESIZE_SCALE = 2.0;
        
        mapping_ = std::make_unique<Mapping>(params);
        
        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&MapperNode::scan_callback, this, std::placeholders::_1));
            
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&MapperNode::odom_callback, this, std::placeholders::_1));
            
        map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 1);
        
        auto meta = nav_msgs::msg::MapMetaData();
        meta.resolution = params.MAP_RESOLUTION;
        meta.width = static_cast<int>(params.MAP_SIZE.first / params.MAP_RESOLUTION);
        meta.height = static_cast<int>(params.MAP_SIZE.second / params.MAP_RESOLUTION);
        meta.origin.position.x = MAP_CENTER_X - params.MAP_SIZE.first / 2.0;
        meta.origin.position.y = MAP_CENTER_Y - params.MAP_SIZE.second / 2.0;
        
        map_msg_.info = meta;
        map_msg_.header.frame_id = "map";
    }

    ~MapperNode() {
        mapping_->save_map();
        RCLCPP_INFO(this->get_logger(), "map save success: %s", mapping_->get_map_filename().c_str());
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_pose_.x = msg->pose.pose.position.x;
        current_pose_.y = msg->pose.pose.position.y;
        
        // 쿼터니언에서 Yaw 추출
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        current_pose_.theta = yaw;
        
        is_odom_ = true;
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        if (!is_map_create_ || !is_odom_) return;
        
        std::vector<double> ranges(msg->ranges.begin(), msg->ranges.end());
        Eigen::MatrixXd laser(2, ranges.size());
        
        for (size_t i = 0; i < ranges.size(); ++i) {
            double angle = msg->angle_min + i * msg->angle_increment;
            if (std::isfinite(ranges[i])) {
                laser(0, i) = ranges[i] * std::cos(angle);
                laser(1, i) = ranges[i] * std::sin(angle);
            } else {
                laser(0, i) = 0.0;
                laser(1, i) = 0.0;
            }
        }
        
        Eigen::Vector3d pose(current_pose_.x, current_pose_.y, current_pose_.theta);
        mapping_->update(pose, laser);
        
        const auto& map_data = mapping_->get_map_data();
        const auto map_size = mapping_->get_map_size();
        std::vector<int8_t> grid_data(map_data.size());
        
        for (size_t i = 0; i < map_data.size(); ++i) {
            int value = static_cast<int>((1.0 - map_data[i]) * 100);
            value = std::min(100, std::max(0, value));
            grid_data[i] = static_cast<int8_t>(value);
        }
        
        map_msg_.header.stamp = now();
        map_msg_.data = grid_data;
        map_pub_->publish(map_msg_);
    }

    void save_map() {
        mapping_->save_map();
    }

private:
    struct RobotPose {
        double x;
        double y;
        double theta;
    } current_pose_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    nav_msgs::msg::OccupancyGrid map_msg_;
    std::unique_ptr<Mapping> mapping_;
    bool is_map_create_;
    bool is_odom_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapperNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 