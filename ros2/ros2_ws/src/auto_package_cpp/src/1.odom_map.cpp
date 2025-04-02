#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Quaternion.h"
#include <opencv2/opencv.hpp>
#include "auto_package_cpp/file_path.hpp"

// 전역 변수로 파일 경로 설정
const std::string MAP_PATH = auto_package_cpp::create_file_path("auto_package_cpp", "map/map.pgm");

class MapOdomNode : public rclcpp::Node {
public:
    MapOdomNode() : Node("map_odom"), initialized_(false) {
        // 맵 로드
        map_image_ = cv::imread(MAP_PATH, cv::IMREAD_GRAYSCALE);
        if(map_image_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load map from %s", MAP_PATH.c_str());
            return;
        }
        
        // scan 토픽 구독
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10,
            std::bind(&MapOdomNode::scan_callback, this, std::placeholders::_1));
        
        // cmd_vel 구독
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10,
            std::bind(&MapOdomNode::cmd_vel_callback, this, std::placeholders::_1));
        
        // Odom 발행자 생성
        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        
        // TF 브로드캐스터 초기화
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        
        // Odometry 메시지 초기화
        odom_.header.frame_id = "map";
        odom_.child_frame_id = "base_link";
        
        // 초기 위치 설정 (원점)
        current_pose_.position.x = 0.0;
        current_pose_.position.y = 0.0;
        current_pose_.position.z = 0.0;
        
        tf2::Quaternion q;
        q.setRPY(0, 0, 0);
        current_pose_.orientation = tf2::toMsg(q);
        current_theta_ = 0.0;

        last_update_time_ = this->now();
    }

private:
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        current_vel_ = *msg;
        last_cmd_vel_time_ = this->now();
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
        if (!initialized_) {
            prev_scan_ = *scan_msg;
            initialized_ = true;
            return;
        }

        auto current_time = this->now();
        double dt = (current_time - last_update_time_).seconds();

        // 1. cmd_vel 기반 예측
        predict_pose(dt);

        // 2. 맵 기반 위치 보정
        auto [correction_x, correction_y, correction_theta] = 
            match_with_map(*scan_msg, current_pose_);

        // 3. 최종 위치 업데이트 (예측과 보정을 결합)
        update_final_pose(correction_x, correction_y, correction_theta);

        // 4. odom 메시지 발행
        publish_odom_and_tf();

        last_update_time_ = current_time;
        prev_scan_ = *scan_msg;
    }

    void predict_pose(double dt) {
        if ((this->now() - last_cmd_vel_time_).seconds() > 0.1) {
            current_vel_.linear.x = 0.0;
            current_vel_.angular.z = 0.0;
        }

        // 관성을 고려한 낮은 신뢰도 적용
        double confidence_factor = 0.3;
        predicted_x_ = current_pose_.position.x + 
            current_vel_.linear.x * dt * cos(current_theta_) * confidence_factor;
        predicted_y_ = current_pose_.position.y + 
            current_vel_.linear.x * dt * sin(current_theta_) * confidence_factor;
        predicted_theta_ = current_theta_ + 
            current_vel_.angular.z * dt * confidence_factor;
    }

    std::tuple<double, double, double> match_with_map(
        const sensor_msgs::msg::LaserScan& scan,
        const geometry_msgs::msg::Pose& predicted_pose) {
        
        std::vector<cv::Point2f> scan_points = convert_scan_to_points(scan);
        
        // 맵에서 현재 예측 위치 주변을 탐색하여 최적의 매칭 찾기
        double best_x = predicted_x_;
        double best_y = predicted_y_;
        double best_theta = predicted_theta_;
        double best_score = std::numeric_limits<double>::max();

        // 간단한 그리드 서치로 최적의 매칭 찾기
        for (double dx = -0.1; dx <= 0.1; dx += 0.02) {
            for (double dy = -0.1; dy <= 0.1; dy += 0.02) {
                for (double dtheta = -0.1; dtheta <= 0.1; dtheta += 0.02) {
                    double score = evaluate_position(scan_points, 
                        predicted_x_ + dx, predicted_y_ + dy, predicted_theta_ + dtheta);
                    
                    if (score < best_score) {
                        best_score = score;
                        best_x = predicted_x_ + dx;
                        best_y = predicted_y_ + dy;
                        best_theta = predicted_theta_ + dtheta;
                    }
                }
            }
        }

        return std::make_tuple(best_x, best_y, best_theta);
    }

    double evaluate_position(const std::vector<cv::Point2f>& scan_points,
                           double x, double y, double theta) {
        double score = 0.0;
        int valid_points = 0;

        for (const auto& point : scan_points) {
            // 스캔 포인트를 맵 좌표계로 변환
            cv::Point2f map_point = transform_to_map(point, x, y, theta);
            
            // 맵 범위 체크
            if (map_point.x < 0 || map_point.x >= map_image_.cols ||
                map_point.y < 0 || map_point.y >= map_image_.rows) {
                continue;
            }

            // 맵에서의 거리값 계산
            uchar map_value = map_image_.at<uchar>(map_point.y, map_point.x);
            score += map_value;
            valid_points++;
        }

        return valid_points > 0 ? score / valid_points : std::numeric_limits<double>::max();
    }

    void update_final_pose(double map_x, double map_y, double map_theta) {
        // 맵 매칭 결과와 예측값을 가중치를 주어 결합
        double map_weight = 0.7;  // 맵 매칭 결과의 가중치
        double pred_weight = 1.0 - map_weight;

        current_pose_.position.x = map_x * map_weight + predicted_x_ * pred_weight;
        current_pose_.position.y = map_y * map_weight + predicted_y_ * pred_weight;
        current_theta_ = map_theta * map_weight + predicted_theta_ * pred_weight;

        tf2::Quaternion q;
        q.setRPY(0, 0, current_theta_);
        current_pose_.orientation = tf2::toMsg(q);
    }

    void publish_odom_and_tf() {
        auto current_time = this->now();

        // Odometry 메시지 업데이트
        odom_.header.stamp = current_time;
        odom_.pose.pose = current_pose_;
        
        // 속도 설정
        odom_.twist.twist = current_vel_;

        // Transform 메시지 생성 및 브로드캐스트
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = current_time;
        tf.header.frame_id = "map";
        tf.child_frame_id = "base_link";
        tf.transform.translation.x = current_pose_.position.x;
        tf.transform.translation.y = current_pose_.position.y;
        tf.transform.translation.z = current_pose_.position.z;
        tf.transform.rotation = current_pose_.orientation;

        // 발행
        tf_broadcaster_->sendTransform(tf);
        odom_publisher_->publish(odom_);
    }

    // 멤버 변수
    cv::Mat map_image_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    nav_msgs::msg::Odometry odom_;
    geometry_msgs::msg::Pose current_pose_;
    sensor_msgs::msg::LaserScan prev_scan_;
    geometry_msgs::msg::Twist current_vel_;
    bool initialized_;
    rclcpp::Time last_update_time_;
    rclcpp::Time last_cmd_vel_time_;
    double current_theta_{0.0};
    double predicted_x_{0.0};
    double predicted_y_{0.0};
    double predicted_theta_{0.0};
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapOdomNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
