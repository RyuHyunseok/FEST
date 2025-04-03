/*
 * Local Dijkstra Path Planner Node
 * 
 * 기능:
 * - 다익스트라 알고리즘을 사용하여 로컬 경로 계획
 * - 전역 경로를 기반으로 로컬 경로 생성
 * - 장애물 회피를 위한 실시간 경로 재계획
 * 
 * 토픽:
 * - 구독:
 *   - /global_path: 전역 경로
 *   - /local_cost_map: 로컬 비용 맵
 *   - /odom: 로봇의 위치 정보
 * - 발행:
 *   - /local_path: 계획된 로컬 경로
 */

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <queue>
#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>
#include <memory>
#include <unordered_map>
#include <chrono>

// using namespace 추가
using namespace std::chrono_literals;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// 다익스트라 알고리즘을 위한 노드 정의
struct DijkstraNode {
    int x, y;
    double cost;
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

class LocalPathPlanner : public rclcpp::Node {
public:
    LocalPathPlanner() : Node("local_path_planner"),
                        has_global_path_(false),
                        has_cost_map_(false),
                        has_odom_(false),
                        lookahead_distance_(10.0),  // 4m 전방 거리 고려
                        collision_threshold_(50)   // 비용 맵에서 70 이상은 충돌로 간주
    {
        // 지역 경로 발행자
        local_path_pub_ = create_publisher<nav_msgs::msg::Path>("local_path", 10);
        
        // 전역 경로 구독자
        global_path_sub_ = create_subscription<nav_msgs::msg::Path>(
            "global_path", 10, 
            std::bind(&LocalPathPlanner::global_path_callback, this, std::placeholders::_1));
        
        // 로컬 코스트맵 구독자
        cost_map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/local_cost_map", 10, 
            std::bind(&LocalPathPlanner::cost_map_callback, this, std::placeholders::_1));
        
        // Odometry 구독자
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, 
            std::bind(&LocalPathPlanner::odom_callback, this, std::placeholders::_1));
        
        // 경로 계획 타이머
        plan_timer_ = create_wall_timer(
            100ms, std::bind(&LocalPathPlanner::plan_path, this));
            
        RCLCPP_INFO(get_logger(), "Local Path Planner initialized");
    }

private:
    void global_path_callback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        global_path_ = *msg;
        has_global_path_ = true;
        RCLCPP_INFO(get_logger(), "Received global path with %zu points", msg->poses.size());
    }

    void cost_map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        cost_map_ = *msg;
        has_cost_map_ = true;
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        odom_ = *msg;
        has_odom_ = true;
        
        // 로봇의 위치 및 방향 계산
        robot_x_ = msg->pose.pose.position.x;
        robot_y_ = msg->pose.pose.position.y;
        
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

    void plan_path()
    {
        if (!has_global_path_ || !has_cost_map_ || !has_odom_) {
            return;
        }

        // 1. 현재 로봇 위치에서 가장 가까운 전역 경로 지점 찾기
        size_t closest_idx = find_closest_point();
        
        // 2. 전역 경로에서 필요한 부분만 추출하여 로컬 경로 생성
        nav_msgs::msg::Path subpath;
        subpath.header.frame_id = "map";
        subpath.header.stamp = now();
        
        // 전역 경로의 가장 가까운 지점부터 일정 거리까지 경로 추출
        double path_length = 0.0;
        bool collision_detected = false;
        size_t collision_idx = 0;
        
        if (closest_idx < global_path_.poses.size()) {
            // 첫 번째 포인트 (가장 가까운 지점) 추가
            subpath.poses.push_back(global_path_.poses[closest_idx]);
            
            // 추가적인 경로 포인트 처리
            for (size_t i = closest_idx + 1; i < global_path_.poses.size(); ++i) {
                // 이전 포인트와의 거리 계산
                double dx = global_path_.poses[i].pose.position.x - 
                            global_path_.poses[i-1].pose.position.x;
                double dy = global_path_.poses[i].pose.position.y - 
                            global_path_.poses[i-1].pose.position.y;
                path_length += std::sqrt(dx*dx + dy*dy);
                
                // 경로 점이 장애물과 충돌하는지 확인
                if (is_point_in_collision(global_path_.poses[i].pose.position.x, 
                                       global_path_.poses[i].pose.position.y)) {
                    collision_detected = true;
                    collision_idx = i;
                    RCLCPP_INFO(get_logger(), "Collision detected at path point %zu", i);
                    break;
                }
                
                // 현재 포인트 추가
                subpath.poses.push_back(global_path_.poses[i]);
                
                // 일정 거리(lookahead_distance_)를 넘어가면 중단
                if (path_length >= lookahead_distance_) {
                    break;
                }
            }
        }
        
        // 3. 충돌이 감지된 경우 다익스트라 알고리즘으로 우회 경로 계획
        if (collision_detected && collision_idx > closest_idx) {
            // 시작점 (충돌 이전 지점)
            double start_x = global_path_.poses[closest_idx].pose.position.x;
            double start_y = global_path_.poses[closest_idx].pose.position.y;
            
            // 목표점 (lookahead_distance_ 근처의 충돌 없는 지점)
            size_t goal_idx = std::min(closest_idx + 100, global_path_.poses.size() - 1);
            double goal_x = global_path_.poses[goal_idx].pose.position.x;
            double goal_y = global_path_.poses[goal_idx].pose.position.y;
            
            // 그리드 좌표로 변환
            int start_grid_x, start_grid_y, goal_grid_x, goal_grid_y;
            world_to_grid(start_x, start_y, start_grid_x, start_grid_y);
            world_to_grid(goal_x, goal_y, goal_grid_x, goal_grid_y);
            
            // 다익스트라 알고리즘으로 경로 계획
            auto path_points = plan_path_dijkstra(start_grid_x, start_grid_y, goal_grid_x, goal_grid_y);
            
            if (!path_points.empty()) {
                // 새 경로 생성
                nav_msgs::msg::Path new_path;
                new_path.header.frame_id = "map";
                new_path.header.stamp = now();
                
                for (const auto& point : path_points) {
                    geometry_msgs::msg::PoseStamped pose;
                    pose.header = new_path.header;
                    
                    // 그리드 좌표를 세계 좌표로 변환
                    double wx, wy;
                    grid_to_world(point.x, point.y, wx, wy);
                    
                    pose.pose.position.x = wx;
                    pose.pose.position.y = wy;
                    pose.pose.position.z = 0.0;
                    pose.pose.orientation.w = 1.0;
                    
                    new_path.poses.push_back(pose);
                }
                
                // 계산된 경로 발행
                local_path_pub_->publish(new_path);
                RCLCPP_INFO(get_logger(), "Published local path with %zu points (Dijkstra)", new_path.poses.size());
            } else {
                // 우회 경로를 찾지 못한 경우, 비어있는 경로 발행
                RCLCPP_WARN(get_logger(), "Failed to find a bypass path");
                nav_msgs::msg::Path empty_path;
                empty_path.header.frame_id = "map";
                empty_path.header.stamp = now();
                local_path_pub_->publish(empty_path);
            }
        } else {
            // 충돌이 없는 경우, 추출한 서브패스 발행
            local_path_pub_->publish(subpath);
            RCLCPP_INFO(get_logger(), "Published local path with %zu points (No collision)", subpath.poses.size());
        }
    }

    size_t find_closest_point()
    {
        if (global_path_.poses.empty()) {
            return 0;
        }
        
        size_t closest_idx = 0;
        double min_dist = std::numeric_limits<double>::max();
        
        for (size_t i = 0; i < global_path_.poses.size(); ++i) {
            double dx = global_path_.poses[i].pose.position.x - robot_x_;
            double dy = global_path_.poses[i].pose.position.y - robot_y_;
            double dist = std::sqrt(dx*dx + dy*dy);
            
            if (dist < min_dist) {
                min_dist = dist;
                closest_idx = i;
            }
        }
        
        return closest_idx;
    }

    bool is_point_in_collision(double x, double y)
    {
        // 세계 좌표를 그리드 좌표로 변환
        int grid_x, grid_y;
        if (!world_to_grid(x, y, grid_x, grid_y)) {
            return true;  // 맵 밖은 충돌로 간주
        }
        
        // 코스트맵에서 비용 확인
        int idx = grid_y * cost_map_.info.width + grid_x;
        if (idx < 0 || idx >= static_cast<int>(cost_map_.data.size())) {
            return true;  // 맵 범위를 벗어나면 충돌로 간주
        }
        
        // 비용이 임계값 이상이면 충돌로 간주
        return (cost_map_.data[idx] >= collision_threshold_);
    }

    bool world_to_grid(double wx, double wy, int &gx, int &gy)
    {
        gx = static_cast<int>((wx - cost_map_.info.origin.position.x) / cost_map_.info.resolution);
        gy = static_cast<int>((wy - cost_map_.info.origin.position.y) / cost_map_.info.resolution);
        
        return (gx >= 0 && gx < static_cast<int>(cost_map_.info.width) &&
                gy >= 0 && gy < static_cast<int>(cost_map_.info.height));
    }

    void grid_to_world(int gx, int gy, double &wx, double &wy)
    {
        wx = gx * cost_map_.info.resolution + cost_map_.info.origin.position.x;
        wy = gy * cost_map_.info.resolution + cost_map_.info.origin.position.y;
    }

    std::vector<DijkstraNode> plan_path_dijkstra(int start_x, int start_y, int goal_x, int goal_y)
    {
        // 다익스트라 알고리즘 구현
        
        // 우선순위 큐 설정 (비용 오름차순)
        auto compare = [](const std::shared_ptr<DijkstraNode>& a, const std::shared_ptr<DijkstraNode>& b) {
            return a->cost > b->cost;
        };
        std::priority_queue<std::shared_ptr<DijkstraNode>, 
                           std::vector<std::shared_ptr<DijkstraNode>>, 
                           decltype(compare)> open_queue(compare);
        
        // 방문 기록 및 비용 맵
        std::unordered_map<DijkstraNode, bool, NodeHash> visited;
        std::unordered_map<DijkstraNode, double, NodeHash> costs;
        
        // 시작 노드 초기화 및 큐에 추가
        auto start_node = std::make_shared<DijkstraNode>(start_x, start_y);
        start_node->cost = 0;
        open_queue.push(start_node);
        costs[*start_node] = 0;
        
        // 방향 배열 (8방향: 상, 우상, 우, 우하, 하, 좌하, 좌, 좌상)
        const int dx[] = {0, 1, 1, 1, 0, -1, -1, -1};
        const int dy[] = {-1, -1, 0, 1, 1, 1, 0, -1};
        const double move_cost[] = {1.0, 1.414, 1.0, 1.414, 1.0, 1.414, 1.0, 1.414};  // 대각선은 √2
        
        while (!open_queue.empty()) {
            // 비용이 가장 낮은 노드 추출
            auto current = open_queue.top();
            open_queue.pop();
            
            // 이미 방문한 노드면 건너뛰기
            if (visited[*current]) continue;
            visited[*current] = true;
            
            // 목표에 도달했는지 확인
            if (current->x == goal_x && current->y == goal_y) {
                return reconstruct_path(current);
            }
            
            // 8방향 이웃 노드 탐색
            for (int i = 0; i < 8; ++i) {
                int nx = current->x + dx[i];
                int ny = current->y + dy[i];
                
                // 맵 범위 내인지 확인
                if (nx < 0 || nx >= static_cast<int>(cost_map_.info.width) ||
                    ny < 0 || ny >= static_cast<int>(cost_map_.info.height)) {
                    continue;
                }
                
                // 인덱스 계산
                int idx = ny * cost_map_.info.width + nx;
                
                // 충돌 여부 확인
                if (cost_map_.data[idx] >= collision_threshold_) {
                    continue;
                }
                
                auto neighbor = std::make_shared<DijkstraNode>(nx, ny);
                
                // 이미 방문한 노드면 건너뛰기
                if (visited[*neighbor]) continue;
                
                // 이동 비용 계산 수정
                double base_cost = cost_map_.data[idx];
                double terrain_cost;
                if (base_cost < 10) {
                    terrain_cost = base_cost * 0.1;  // 낮은 비용 영역
                } else if (base_cost < 20) {
                    terrain_cost = base_cost * 0.2;  // 중간 비용 영역
                } else {
                    terrain_cost = base_cost * 0.3;  // 높은 비용 영역
                }
                
                // 휴리스틱 비용 추가
                double dx_to_goal = goal_x - nx;
                double dy_to_goal = goal_y - ny;
                double heuristic = std::sqrt(dx_to_goal*dx_to_goal + dy_to_goal*dy_to_goal) * 0.1;
                
                double new_cost = current->cost + move_cost[i] * (1.0 + terrain_cost) + heuristic;
                
                // 더 나은 경로를 발견하거나 처음 방문하는 노드인 경우
                auto cost_it = costs.find(*neighbor);
                if (cost_it == costs.end() || new_cost < cost_it->second) {
                    costs[*neighbor] = new_cost;
                    neighbor->cost = new_cost;
                    neighbor->parent = current;
                    open_queue.push(neighbor);
                }
            }
        }
        
        // 경로를 찾지 못한 경우
        return std::vector<DijkstraNode>();
    }

    std::vector<DijkstraNode> reconstruct_path(std::shared_ptr<DijkstraNode> goal)
    {
        std::vector<DijkstraNode> path;
        auto current = goal;
        
        while (current != nullptr) {
            path.push_back(*current);
            current = current->parent;
        }
        
        std::reverse(path.begin(), path.end());
        return path;
    }

    // Publishers
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_;
    
    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr cost_map_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr plan_timer_;
    
    // Data
    nav_msgs::msg::Path global_path_;
    nav_msgs::msg::OccupancyGrid cost_map_;
    nav_msgs::msg::Odometry odom_;
    
    // Robot state
    double robot_x_;
    double robot_y_;
    double robot_yaw_;
    
    // Parameters
    double lookahead_distance_;  // 경로 계획 거리
    int collision_threshold_;    // 충돌 판단 임계값
    
    // State flags
    bool has_global_path_;
    bool has_cost_map_;
    bool has_odom_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LocalPathPlanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 