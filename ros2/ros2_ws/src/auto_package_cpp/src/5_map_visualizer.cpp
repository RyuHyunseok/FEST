#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/map_meta_data.hpp>
#include <fstream>
#include <vector>
#include <string>
#include <cmath>

class MapVisualizerNode : public rclcpp::Node {
public:
    MapVisualizerNode() : Node("map_visualizer") {
        // 맵 파일 경로 설정
        map_filename_ = R"(C:\Users\SSAFY\Desktop\S12P21D106\ros2\ros2_ws\src\auto_package_cpp\path\map.pgm)";
        cost_map_filename_ = R"(C:\Users\SSAFY\Desktop\S12P21D106\ros2\ros2_ws\src\auto_package_cpp\path\cost_map.pgm)";
        
        // 파라미터 설정
        inflation_radius_ = 0.3;  // 30cm
        cost_scaling_factor_ = 2.0;
        
        // 맵 메타데이터 설정
        map_meta_.resolution = 0.05;  // 5cm per pixel
        map_meta_.width = 400;        // 20m / 0.05m = 400 pixels
        map_meta_.height = 400;       // 20m / 0.05m = 400 pixels
        map_meta_.origin.position.x = -10.0;  // -20m/2
        map_meta_.origin.position.y = -10.0;  // -20m/2
        
        // 맵 메시지 설정
        map_msg_.info = map_meta_;
        map_msg_.header.frame_id = "map";
        
        cost_map_msg_ = map_msg_;  // Cost Map 메시지 초기화
        
        // 발행자 생성
        map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 1);
        cost_map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/cost_map", 1);
        
        // 타이머 생성 (1Hz로 맵 발행)
        timer_ = create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&MapVisualizerNode::timer_callback, this));
            
        RCLCPP_INFO(this->get_logger(), "map_visualizer start.");
    }

    ~MapVisualizerNode() {
        // 노드 종료 시 Cost Map 저장
        saveCostMap();
        RCLCPP_INFO(this->get_logger(), "Cost map saved to: %s", cost_map_filename_.c_str());
    }

private:
    void timer_callback() {
        if (load_map()) {
            // 기본 맵 발행
            map_msg_.header.stamp = now();
            map_pub_->publish(map_msg_);

            // Cost Map 생성 및 발행
            generateCostMap();
            cost_map_msg_.header.stamp = now();
            cost_map_pub_->publish(cost_map_msg_);
        }
    }

    bool load_map() {
        std::ifstream file(map_filename_, std::ios::binary);
        if (!file) {
            RCLCPP_ERROR(this->get_logger(), "map file open error: %s", map_filename_.c_str());
            return false;
        }

        // PGM 헤더 읽기
        std::string magic;
        file >> magic;
        if (magic != "P5") {
            RCLCPP_ERROR(this->get_logger(), "wrong pgm file format.");
            return false;
        }

        int width, height, maxval;
        file >> width >> height >> maxval;
        file.ignore(1);  // 개행 문자 건너뛰기

        if (width != map_meta_.width || height != map_meta_.height) {
            RCLCPP_ERROR(this->get_logger(), "map size is not same.");
            return false;
        }

        // 맵 데이터 읽기
        std::vector<unsigned char> buffer(width * height);
        file.read(reinterpret_cast<char*>(buffer.data()), buffer.size());

        // OccupancyGrid 데이터로 변환
        map_msg_.data.resize(buffer.size());
        for (size_t i = 0; i < buffer.size(); ++i) {
            map_msg_.data[i] = static_cast<int8_t>((buffer[i] * 100 / 255));
        }

        return true;
    }

    void generateCostMap() {
        cost_map_msg_.data = map_msg_.data;  // 기본 맵 복사
        
        // 장애물 팽창
        int inflation_cells = static_cast<int>(inflation_radius_ / map_meta_.resolution);
        
        std::vector<int8_t> temp_map = cost_map_msg_.data;
        
        for (int y = 0; y < map_meta_.height; ++y) {
            for (int x = 0; x < map_meta_.width; ++x) {
                if (map_msg_.data[y * map_meta_.width + x] > 90) {  // 장애물인 경우
                    // 주변 셀에 비용 할당
                    for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
                        for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
                            int new_x = x + dx;
                            int new_y = y + dy;
                            
                            if (new_x >= 0 && new_x < map_meta_.width &&
                                new_y >= 0 && new_y < map_meta_.height) {
                                
                                double distance = std::hypot(dx * map_meta_.resolution, 
                                                           dy * map_meta_.resolution);
                                
                                if (distance <= inflation_radius_) {
                                    int cost = static_cast<int>(
                                        90 * std::exp(-cost_scaling_factor_ * distance));
                                    
                                    int index = new_y * map_meta_.width + new_x;
                                    temp_map[index] = std::max(temp_map[index], 
                                                             static_cast<int8_t>(cost));
                                }
                            }
                        }
                    }
                }
            }
        }
        
        cost_map_msg_.data = temp_map;
    }

    void saveCostMap() {
        std::ofstream file(cost_map_filename_, std::ios::binary);
        if (!file) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open cost map file for writing");
            return;
        }

        // PGM 헤더 쓰기
        file << "P5\n" << map_meta_.width << " " << map_meta_.height << "\n255\n";

        // Cost Map 데이터를 PGM 형식으로 변환하여 저장
        std::vector<unsigned char> buffer(cost_map_msg_.data.size());
        for (size_t i = 0; i < cost_map_msg_.data.size(); ++i) {
            buffer[i] = static_cast<unsigned char>(255 - (cost_map_msg_.data[i] * 255 / 100));
        }

        file.write(reinterpret_cast<char*>(buffer.data()), buffer.size());
    }

    std::string map_filename_;
    std::string cost_map_filename_;
    nav_msgs::msg::MapMetaData map_meta_;
    nav_msgs::msg::OccupancyGrid map_msg_;
    nav_msgs::msg::OccupancyGrid cost_map_msg_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr cost_map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    double inflation_radius_;
    double cost_scaling_factor_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapVisualizerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 