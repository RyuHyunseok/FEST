#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <opencv2/opencv.hpp>
#include "common_utils/file_path.hpp"

// 전역 변수 선언 및 초기화
std::string MAP_FILE = common_utils::create_file_path("perception_cpp", "map/map_save.pgm");

class MappingNode : public rclcpp::Node
{
public:
    MappingNode() : Node("mapping_node")
    {
        // 파라미터 초기화
        this->declare_parameter("map_resolution", 0.05);  // 미터/픽셀
        this->declare_parameter("map_width", 2000);       // 픽셀
        this->declare_parameter("map_height", 2000);      // 픽셀
        
        resolution_ = this->get_parameter("map_resolution").as_double();
        width_ = this->get_parameter("map_width").as_int();
        height_ = this->get_parameter("map_height").as_int();

        // 맵 초기화
        map_ = cv::Mat(height_, width_, CV_8U, cv::Scalar(50));  // 50: 미확인 영역
        map_origin_x_ = -width_ * resolution_ / 2.0;
        map_origin_y_ = -height_ * resolution_ / 2.0;

        // TF 버퍼 및 리스너 초기화
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 구독자 생성
        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&MappingNode::scan_callback, this, std::placeholders::_1));

        // 발행자 생성
        map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

        // 맵 발행 타이머
        map_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&MappingNode::publish_map, this));

        // 맵 저장 타이머 추가 (10초마다 저장)
        save_timer_ = this->create_wall_timer(
            std::chrono::seconds(10),
            std::bind(&MappingNode::save_map, this));

        RCLCPP_INFO(this->get_logger(), "Mapping node started");
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        try {
            // 현재 로봇의 위치 얻기
            geometry_msgs::msg::TransformStamped transform = 
                tf_buffer_->lookupTransform("map", "base_link", msg->header.stamp);

            double robot_x = transform.transform.translation.x;
            double robot_y = transform.transform.translation.y;
            double robot_yaw = 2 * std::atan2(transform.transform.rotation.z, 
                                            transform.transform.rotation.w);

            // 로봇 위치를 맵 좌표계로 변환
            int robot_map_x = (robot_x - map_origin_x_) / resolution_;
            int robot_map_y = (robot_y - map_origin_y_) / resolution_;

            // 라이다 스캔 데이터를 맵에 업데이트
            for (size_t i = 0; i < msg->ranges.size(); i++)
            {
                float range = msg->ranges[i];
                if (!std::isfinite(range)) continue;
                if (range < msg->range_min || range > msg->range_max) continue;

                double angle = msg->angle_min + i * msg->angle_increment + robot_yaw;
                double point_x = robot_x + range * std::cos(angle);
                double point_y = robot_y + range * std::sin(angle);

                int map_x = (point_x - map_origin_x_) / resolution_;
                int map_y = (point_y - map_origin_y_) / resolution_;

                if (map_x >= 0 && map_x < width_ && map_y >= 0 && map_y < height_)
                {
                    // Bresenham 알고리즘으로 로봇에서 측정점까지 레이 캐스팅
                    bresenham_line(robot_map_x, robot_map_y, map_x, map_y);
                    // 측정된 점은 장애물로 표시
                    map_.at<uchar>(map_y, map_x) = 100;
                }
            }
        }
        catch (tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(), "Transform failed: %s", ex.what());
        }
    }

    void bresenham_line(int x1, int y1, int x2, int y2)
    {
        int dx = std::abs(x2 - x1);
        int dy = std::abs(y2 - y1);
        int sx = (x1 < x2) ? 1 : -1;
        int sy = (y1 < y2) ? 1 : -1;
        int err = dx - dy;

        while (true)
        {
            if (x1 >= 0 && x1 < width_ && y1 >= 0 && y1 < height_)
            {
                // 빈 공간으로 표시
                map_.at<uchar>(y1, x1) = 0;
            }

            if (x1 == x2 && y1 == y2) break;

            int e2 = 2 * err;
            if (e2 > -dy)
            {
                err -= dy;
                x1 += sx;
            }
            if (e2 < dx)
            {
                err += dx;
                y1 += sy;
            }
        }
    }

    void publish_map()
    {
        auto map_msg = nav_msgs::msg::OccupancyGrid();
        map_msg.header.stamp = this->now();
        map_msg.header.frame_id = "map";
        map_msg.info.resolution = resolution_;
        map_msg.info.width = width_;
        map_msg.info.height = height_;
        map_msg.info.origin.position.x = map_origin_x_;
        map_msg.info.origin.position.y = map_origin_y_;

        map_msg.data.resize(width_ * height_);
        for (int y = 0; y < height_; y++)
        {
            for (int x = 0; x < width_; x++)
            {
                map_msg.data[y * width_ + x] = map_.at<uchar>(y, x);
            }
        }

        map_pub_->publish(map_msg);
    }

    void save_map()
    {
        try {
            cv::imwrite(MAP_FILE, map_);
            RCLCPP_INFO(this->get_logger(), "Map saved to: %s", MAP_FILE.c_str());
        } catch (const cv::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to save map: %s", e.what());
        }
    }

    // 맵 관련 변수
    cv::Mat map_;
    double resolution_;
    int width_;
    int height_;
    double map_origin_x_;
    double map_origin_y_;

    // ROS 관련 변수
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr map_timer_;
    rclcpp::TimerBase::SharedPtr save_timer_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MappingNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 