/*
 * Global Dijkstra Path Planner Node
 * 
 * 기능:
 * - 다익스트라 알고리즘을 사용하여 전역 경로 계획
 * - 비용 맵을 기반으로 최적의 경로 생성
 * - MQTT를 통한 목표점 수신
 * 
 * 토픽:
 * - 구독:
 *   - /cost_map: 비용 맵 데이터
 *   - /odom: 로봇의 위치 정보
 *   - /goal_point: 목표점 정보
 * - 발행:
 *   - /global_path: 계획된 전역 경로
 *   - /visualization_marker_array: 경로 시각화 마커
 */

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
// #include "auto_package_cpp/mqtt_handler.hpp"
#include <queue>
#include <vector>
#include <cmath>
#include <algorithm>
#include <memory>
#include <unordered_map>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

struct DijkstraNode {
    int x, y;
    double cost;  // 시작점으로부터의 누적 비용
    std::shared_ptr<DijkstraNode> parent;

    DijkstraNode(int _x, int _y) : x(_x), y(_y), cost(0), parent(nullptr) {}

    bool operator==(const DijkstraNode& other) const {
        return x == other.x && y == other.y;
    }
};

// 해시 함수 정의
struct NodeHash {
    std::size_t operator()(const DijkstraNode& node) const {
        return std::hash<int>()(node.x) ^ (std::hash<int>()(node.y) << 1);
    }
};

class DijkstraPlanner : public rclcpp::Node {
public:
    DijkstraPlanner() : Node("dijkstra_planner"), has_map_(false), has_robot_pose_(false), has_goal_(false) {
        // 구독자 생성
        map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/cost_map", 1, std::bind(&DijkstraPlanner::map_callback, this, std::placeholders::_1));
        
        goal_sub_ = create_subscription<geometry_msgs::msg::Point>(
            "/goal_point", 10, std::bind(&DijkstraPlanner::goal_callback, this, std::placeholders::_1));
        
        // // =============================================
        // // MQTT 핸들러 초기화 및 설정
        // // =============================================
        
        // // MQTT 브로커 연결 설정 (localhost:1883)
        // mqtt_handler_ = std::make_unique<MqttHandler>("localhost", 1883);
        
        // // MQTT 연결 시도 및 콜백 설정
        // if (mqtt_handler_->connect()) {
        //     // 목표점 수신 시 실행될 콜백 함수 설정
        //     mqtt_handler_->set_goal_callback([this](double x, double y) {
        //         // 수신된 좌표를 geometry_msgs::Point 메시지로 변환
        //         geometry_msgs::msg::Point goal_point;
        //         goal_point.x = x;
        //         goal_point.y = y;
        //         // goal_callback 함수 호출하여 경로 계획 시작
        //         goal_callback(std::make_shared<geometry_msgs::msg::Point>(goal_point));
        //     });
        //     RCLCPP_INFO(get_logger(), "MQTT handler initialized successfully");
        // } else {    // MQTT 연결 실패 시 대체 처리
        //     // ROS2 토픽을 통한 목표점 수신 설정
        //     goal_sub_ = create_subscription<geometry_msgs::msg::Point>(
        //         "/goal_point", 10,
        //         std::bind(&DijkstraPlanner::goal_callback, this, std::placeholders::_1));
        //     RCLCPP_WARN(get_logger(), "MQTT connection failed, using ROS2 topic /goal_point");
        // }

        // // =============================================
        // // MQTT 또는 ROS2 토픽을 통한 목표점 수신 설정 종료
        // // =============================================

        // 로봇의 실제 위치를 받기 위한 구독자 추가
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&DijkstraPlanner::odom_callback, this, std::placeholders::_1));

        // 발행자 생성
        path_pub_ = create_publisher<nav_msgs::msg::Path>("/global_path", 1);
        marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/planning_points", 1);

        RCLCPP_INFO(get_logger(), "Dijkstra Planner initialized");
    }

private:
    // 클래스 멤버 변수 선언
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr goal_sub_;  // ROS2 토픽 구독자
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    // std::unique_ptr<MqttHandler> mqtt_handler_;
    nav_msgs::msg::OccupancyGrid map_;
    bool has_map_;
    bool has_robot_pose_;
    bool has_goal_;
    
    // 로봇의 초기 위치와 현재 위치 저장
    double robot_init_x_;
    double robot_init_y_;
    double robot_init_theta_;
    double current_robot_x_;
    double current_robot_y_;
    double current_robot_theta_;
    
    // 목표점 저장
    int goal_x_;
    int goal_y_;

    void goal_callback(const geometry_msgs::msg::Point::SharedPtr msg) {
        // 목표점을 맵 좌표계로 변환
        goal_x_ = static_cast<int>((msg->x - map_.info.origin.position.x) / map_.info.resolution);
        goal_y_ = static_cast<int>((msg->y - map_.info.origin.position.y) / map_.info.resolution);
        has_goal_ = true;
        RCLCPP_INFO(get_logger(), "Received new goal point: map coordinates (%d, %d)", goal_x_, goal_y_);
        
        // 새로운 목표점이 설정되면 경로 계획 실행
        plan_current_path();
    }

    void plan_current_path() {
        if (!has_map_ || !has_robot_pose_ || !has_goal_) {
            RCLCPP_WARN(get_logger(), "Waiting for map, robot pose, and goal...");
            return;
        }

        // 로봇의 현재 위치를 맵 좌표로 변환하여 시작점으로 설정
        int start_x = static_cast<int>((current_robot_x_ - map_.info.origin.position.x) / map_.info.resolution);
        int start_y = static_cast<int>((current_robot_y_ - map_.info.origin.position.y) / map_.info.resolution);
        
        // 시작점과 목표점의 cost 값 확인
        int start_index = start_y * map_.info.width + start_x;
        int goal_index = goal_y_ * map_.info.width + goal_x_;
        
        if (start_index < 0 || start_index >= static_cast<int>(map_.data.size()) ||
            goal_index < 0 || goal_index >= static_cast<int>(map_.data.size())) {
            RCLCPP_ERROR(get_logger(), "Start or goal position is outside map bounds");
            return;
        }
        
        int start_cost = map_.data[start_index];
        int goal_cost = map_.data[goal_index];
        
        RCLCPP_INFO(get_logger(), "Start position (%d, %d) cost: %d, Goal position (%d, %d) cost: %d", 
                    start_x, start_y, start_cost, goal_x_, goal_y_, goal_cost);
        
        // 시작점과 목표점 설정
        DijkstraNode start(start_x, start_y);
        DijkstraNode goal(goal_x_, goal_y_);

        // 시작점과 목표점 마커 발행
        publish_points_marker(start, goal);

        auto path = plan_path(start, goal);
        if (!path.empty()) {
            publish_path(path);
            RCLCPP_INFO(get_logger(), "Path published with %zu points", path.size());
        } else {
            RCLCPP_WARN(get_logger(), "No path found between (%d, %d) and (%d, %d)", 
                        start_x, start_y, goal_x_, goal_y_);
        }
    }

    //int map_update_count_ = 0;  // 맵 업데이트 횟수 추적

    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        map_ = *msg;
        has_map_ = true;
        //map_update_count_++;
        
        std::string status_info = "Costmap received: " + std::to_string(map_.info.width) + "x" + std::to_string(map_.info.height);
        
        if (has_robot_pose_) {
            int current_x = static_cast<int>((current_robot_x_ - map_.info.origin.position.x) / map_.info.resolution);
            int current_y = static_cast<int>((current_robot_y_ - map_.info.origin.position.y) / map_.info.resolution);
            status_info += " | Robot Position: (" + std::to_string(current_x) + ", " + std::to_string(current_y) + ")";
        }
        
        if (has_goal_) {
            status_info += " | Goal Position: (" + std::to_string(goal_x_) + ", " + std::to_string(goal_y_) + ")";
        }
        
        RCLCPP_INFO(get_logger(), "%s", status_info.c_str());
        
        // 맵이 5번 업데이트될 때마다 경로 재계획
        //if (has_goal_ && map_update_count_ % 5 == 0) {
        // if (has_goal_) {
        //     plan_current_path();
        // }
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_robot_x_ = msg->pose.pose.position.x;
        current_robot_y_ = msg->pose.pose.position.y;
        has_robot_pose_ = true;
    }

    void publish_points_marker(const DijkstraNode& start, const DijkstraNode& goal) {
        visualization_msgs::msg::MarkerArray marker_array;
        
        // 시작점 마커
        visualization_msgs::msg::Marker start_marker;
        start_marker.header.frame_id = "map";
        start_marker.header.stamp = now();
        start_marker.ns = "planning_points";
        start_marker.id = 0;
        start_marker.type = visualization_msgs::msg::Marker::SPHERE;
        start_marker.action = visualization_msgs::msg::Marker::ADD;
        
        start_marker.pose.position.x = start.x * map_.info.resolution + map_.info.origin.position.x;
        start_marker.pose.position.y = start.y * map_.info.resolution + map_.info.origin.position.y;
        start_marker.pose.position.z = 0.0;
        start_marker.pose.orientation.w = 1.0;
        
        start_marker.scale.x = 0.3;
        start_marker.scale.y = 0.3;
        start_marker.scale.z = 0.3;
        
        start_marker.color.r = 0.0;
        start_marker.color.g = 1.0;
        start_marker.color.b = 0.0;
        start_marker.color.a = 1.0;
        
        // 목표점 마커
        visualization_msgs::msg::Marker goal_marker = start_marker;
        goal_marker.id = 1;
        goal_marker.pose.position.x = goal.x * map_.info.resolution + map_.info.origin.position.x;
        goal_marker.pose.position.y = goal.y * map_.info.resolution + map_.info.origin.position.y;
        goal_marker.color.r = 1.0;
        goal_marker.color.g = 0.0;
        goal_marker.color.b = 0.0;
        
        marker_array.markers.push_back(start_marker);
        marker_array.markers.push_back(goal_marker);
        
        marker_pub_->publish(marker_array);
    }

    std::vector<DijkstraNode> plan_path(const DijkstraNode& start, const DijkstraNode& goal) {
        // 우선순위 큐 (비용이 가장 낮은 노드가 먼저 처리됨)
        auto compare = [](const std::shared_ptr<DijkstraNode>& a, const std::shared_ptr<DijkstraNode>& b) {
            return a->cost > b->cost;
        };
        std::priority_queue<std::shared_ptr<DijkstraNode>, 
                          std::vector<std::shared_ptr<DijkstraNode>>, 
                          decltype(compare)> open_queue(compare);

        // 방문한 노드 추적
        std::unordered_map<DijkstraNode, bool, NodeHash> visited;
        std::unordered_map<DijkstraNode, double, NodeHash> costs;

        // 시작점 초기화
        auto start_node = std::make_shared<DijkstraNode>(start);
        start_node->cost = 0;
        open_queue.push(start_node);
        costs[*start_node] = 0;

        int max_iterations = 100000;  // 최대 반복 횟수
        int current_iteration = 0;
        int nodes_expanded = 0;

        // 방향 배열 (8방향: 상하좌우 + 대각선)
        const int dx[] = {-1, -1, -1, 0, 0, 1, 1, 1};
        const int dy[] = {-1, 0, 1, -1, 1, -1, 0, 1};
        const double move_cost[] = {1.414, 1.0, 1.414, 1.0, 1.0, 1.414, 1.0, 1.414};

        while (!open_queue.empty() && current_iteration < max_iterations) {
            current_iteration++;
            
            // 비용이 가장 낮은 노드 꺼내기
            auto current = open_queue.top();
            open_queue.pop();

            // 목표에 도달했는지 확인
            if (current->x == goal.x && current->y == goal.y) {
                RCLCPP_INFO(get_logger(), 
                    "Path found after %d iterations, expanded %d nodes", 
                    current_iteration, nodes_expanded);
                return reconstruct_path(current);
            }

            // 이미 방문한 노드는 건너뛰기
            if (visited[*current]) continue;
            visited[*current] = true;
            nodes_expanded++;

            // 8방향 이웃 노드 탐색
            for (int i = 0; i < 8; i++) {
                int new_x = current->x + dx[i];
                int new_y = current->y + dy[i];

                // 유효한 위치인지 확인
                if (!is_valid_position(new_x, new_y)) {
                    continue;
                }

                // 비용 패널티 계산
                double cost_penalty = get_cost_penalty(new_x, new_y);
                if (cost_penalty < 0) {
                    // 장애물인 경우 건너뛰기
                    continue;
                }

                auto neighbor = std::make_shared<DijkstraNode>(new_x, new_y);
                
                // 이미 방문한 노드는 건너뛰기
                if (visited[*neighbor]) continue;

                // 이동 비용 계산
                double new_cost = current->cost + move_cost[i] * (1.0 + cost_penalty);

                // 더 나은 경로를 찾았거나 이전에 방문하지 않은 노드인 경우
                auto cost_it = costs.find(*neighbor);
                if (cost_it == costs.end() || new_cost < cost_it->second) {
                    costs[*neighbor] = new_cost;
                    neighbor->cost = new_cost;
                    neighbor->parent = current;
                    open_queue.push(neighbor);
                }
            }

            // 디버그 정보
            if (current_iteration % 1000 == 0) {
                RCLCPP_DEBUG(get_logger(), 
                    "Iteration %d: Expanded %d nodes, Queue size: %zu", 
                    current_iteration, nodes_expanded, open_queue.size());
            }
        }

        if (current_iteration >= max_iterations) {
            RCLCPP_WARN(get_logger(), 
                "Path planning exceeded maximum iterations (%d), expanded %d nodes", 
                max_iterations, nodes_expanded);
        }

        // 경로를 찾지 못함
        return std::vector<DijkstraNode>();
    }

    double get_cost_penalty(int x, int y) {
        if (!has_map_) return -1;
        int index = y * map_.info.width + x;
        if (index < 0 || index >= static_cast<int>(map_.data.size())) {
            RCLCPP_ERROR(get_logger(), "Invalid map index: %d (max: %zu)", 
                        index, map_.data.size());
            return -1;
        }
        
        int cost = map_.data[index];
        
        // 완전한 장애물(100)만 통과 불가능으로 처리
        if (cost >= 65) return -1;
        
        // cost가 낮을수록(0) 낮은 패널티
        // cost가 높을수록(100에 가까울수록) 높은 패널티
        if (cost <= 1) {  // 낮은 cost (통로)
            return 0.1;   // 낮은 패널티
        } else {
            // 중간 영역은 비례적으로 패널티 계산
            // cost가 높을수록 패널티가 높아짐
            return (4.9 * cost / 100.0);
        }
    }

    bool is_valid_position(int x, int y) {
        return x >= 0 && x < static_cast<int>(map_.info.width) &&
               y >= 0 && y < static_cast<int>(map_.info.height);
    }

    std::vector<DijkstraNode> reconstruct_path(std::shared_ptr<DijkstraNode> goal_node) {
        std::vector<DijkstraNode> path;
        auto current = goal_node;

        while (current != nullptr) {
            path.push_back(*current);
            current = current->parent;
        }

        std::reverse(path.begin(), path.end());
        return path;
    }

    void publish_path(const std::vector<DijkstraNode>& path) {
        nav_msgs::msg::Path path_msg;
        path_msg.header.frame_id = "map";
        path_msg.header.stamp = now();

        for (const auto& node : path) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path_msg.header;
            
            pose.pose.position.x = node.x * map_.info.resolution + map_.info.origin.position.x;
            pose.pose.position.y = node.y * map_.info.resolution + map_.info.origin.position.y;
            pose.pose.orientation.w = 1.0;

            path_msg.poses.push_back(pose);
        }

        path_pub_->publish(path_msg);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DijkstraPlanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}