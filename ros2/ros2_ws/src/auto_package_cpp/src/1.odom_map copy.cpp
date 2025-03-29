#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Quaternion.h"

class SlamOdomNode : public rclcpp::Node {
public:
    SlamOdomNode() : Node("slam_odom"), initialized_(false) {
        // scan 토픽 구독
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10,
            std::bind(&SlamOdomNode::scan_callback, this, std::placeholders::_1));
        
        // Odom 발행자 생성
        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        
        // TF 브로드캐스터 초기화
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        
        // Odometry 메시지 초기화
        odom_.header.frame_id = "map";
        odom_.child_frame_id = "base_link";
        
        // 초기 위치 설정
        current_pose_.position.x = 0.0;
        current_pose_.position.y = 0.0;
        current_pose_.position.z = 0.0;
        
        tf2::Quaternion q;
        q.setRPY(0, 0, 0);
        current_pose_.orientation.x = q.x();
        current_pose_.orientation.y = q.y();
        current_pose_.orientation.z = q.z();
        current_pose_.orientation.w = q.w();

        // cmd_vel 구독 추가
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10,
            std::bind(&SlamOdomNode::cmd_vel_callback, this, std::placeholders::_1));

        last_update_time_ = this->now();
    }

private:
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        current_vel_ = *msg;
        last_cmd_vel_time_ = this->now();
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
        auto current_time = this->now();
        
        if (!initialized_) {
            prev_scan_ = *scan_msg;
            initialized_ = true;
            last_update_time_ = current_time;
            return;
        }

        // 속도 기반 예측 단계
        double dt = (current_time - last_update_time_).seconds();
        predict_pose(dt);

        // 스캔 매칭으로 보정 단계
        auto [scan_dx, scan_dy, scan_dtheta] = calculate_transform(prev_scan_, *scan_msg);
        update_pose(scan_dx, scan_dy, scan_dtheta);

        publish_odom_and_tf();
        
        prev_scan_ = *scan_msg;
        last_update_time_ = current_time;
    }

    void predict_pose(double dt) {
        if ((this->now() - last_cmd_vel_time_).seconds() > 0.1) {
            // cmd_vel 시간 초과시 속도 0으로 설정
            current_vel_.linear.x = 0.0;
            current_vel_.angular.z = 0.0;
        }
        
        // cmd_vel은 참고용으로만 사용
        double confidence_factor = 0.3;  // cmd_vel 신뢰도
        double predicted_x = current_pose_.position.x + 
            current_vel_.linear.x * dt * cos(current_theta_) * confidence_factor;
        double predicted_y = current_pose_.position.y + 
            current_vel_.linear.x * dt * sin(current_theta_) * confidence_factor;
        double predicted_theta = current_theta_ + 
            current_vel_.angular.z * dt * confidence_factor;
        
        // 예측값 적용
        update_prediction(predicted_x, predicted_y, predicted_theta);
    }

    std::tuple<double, double, double> calculate_transform(
        const sensor_msgs::msg::LaserScan& prev_scan,
        const sensor_msgs::msg::LaserScan& current_scan) {
        // ICP 알고리즘 구현
        std::vector<Point2D> prev_points = convert_scan_to_points(prev_scan);
        std::vector<Point2D> current_points = convert_scan_to_points(current_scan);
        
        // ICP 반복 실행
        double dx = 0.0, dy = 0.0, dtheta = 0.0;
        for(int iter = 0; iter < MAX_ITERATIONS; iter++) {
            // 1. 가장 가까운 점들 매칭
            // 2. 변환 계산
            // 3. 에러 계산 및 수렴 확인
        }
        
        return std::make_tuple(dx, dy, dtheta);
    }

    void update_pose(double scan_dx, double scan_dy, double scan_dtheta) {
        // 스캔 매칭과 속도 예측을 가중치 기반으로 융합
        double scan_weight = calculate_scan_confidence();  // 스캔 매칭 신뢰도
        double vel_weight = 1.0 - scan_weight;  // 속도 기반 예측 신뢰도
        
        double final_dx = scan_dx * scan_weight + predicted_dx_ * vel_weight;
        double final_dy = scan_dy * scan_weight + predicted_dy_ * vel_weight;
        double final_dtheta = scan_dtheta * scan_weight + predicted_dtheta_ * vel_weight;
        
        apply_pose_update(final_dx, final_dy, final_dtheta);
    }

    void publish_odom_and_tf() {
        auto current_time = this->now();

        // Odometry 메시지 업데이트
        odom_.header.stamp = current_time;
        odom_.pose.pose = current_pose_;
        
        // 속도 추정 (간단한 예시)
        odom_.twist.twist.linear.x = 0.0;  // 실제로는 변위로부터 계산
        odom_.twist.twist.angular.z = 0.0;

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

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    nav_msgs::msg::Odometry odom_;
    geometry_msgs::msg::Pose current_pose_;
    sensor_msgs::msg::LaserScan prev_scan_;
    bool initialized_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    geometry_msgs::msg::Twist current_vel_;
    rclcpp::Time last_update_time_;
    rclcpp::Time last_cmd_vel_time_;
    double current_theta_{0.0};
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SlamOdomNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
