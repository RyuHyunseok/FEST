#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <vector>
#include <cmath>
#include <Eigen/Dense>

class SlamOdometry : public rclcpp::Node
{
public:
    SlamOdometry() : Node("slam_odometry")
    {
        // 구독자 생성
        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&SlamOdometry::scan_callback, this, std::placeholders::_1));

        // 발행자 생성
        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        
        // TF 브로드캐스터 초기화
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // 초기 위치 설정
        current_x_ = 0.0;
        current_y_ = 0.0;
        current_theta_ = 0.0;

        // 이전 스캔 데이터 초기화 플래그
        is_first_scan_ = true;

        RCLCPP_INFO(this->get_logger(), "SLAM Odometry node started");
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        if (is_first_scan_)
        {
            previous_scan_ = *msg;
            is_first_scan_ = false;
            return;
        }

        // 스캔 매칭 수행
        auto result = match_scans(previous_scan_, *msg);
        double dx = std::get<0>(result);
        double dy = std::get<1>(result);
        double dtheta = std::get<2>(result);

        // 위치 업데이트
        current_x_ += dx * cos(current_theta_) - dy * sin(current_theta_);
        current_y_ += dx * sin(current_theta_) + dy * cos(current_theta_);
        current_theta_ += dtheta;

        // 각도 정규화 (-π ~ π)
        current_theta_ = std::atan2(sin(current_theta_), cos(current_theta_));

        // 오도메트리 메시지 발행
        publish_odometry(msg->header.stamp);

        // 현재 스캔을 이전 스캔으로 저장
        previous_scan_ = *msg;
    }

    std::tuple<double, double, double> match_scans(
        const sensor_msgs::msg::LaserScan& prev_scan,
        const sensor_msgs::msg::LaserScan& current_scan)
    {
        std::vector<Eigen::Vector2d> prev_points;
        std::vector<Eigen::Vector2d> current_points;

        // 스캔 데이터를 포인트로 변환
        for (size_t i = 0; i < prev_scan.ranges.size(); i++)
        {
            float range = prev_scan.ranges[i];
            if (std::isfinite(range) && range >= prev_scan.range_min && 
                range <= prev_scan.range_max)
            {
                double angle = prev_scan.angle_min + i * prev_scan.angle_increment;
                prev_points.emplace_back(
                    range * cos(angle),
                    range * sin(angle)
                );
            }
        }

        for (size_t i = 0; i < current_scan.ranges.size(); i++)
        {
            float range = current_scan.ranges[i];
            if (std::isfinite(range) && range >= current_scan.range_min && 
                range <= current_scan.range_max)
            {
                double angle = current_scan.angle_min + i * current_scan.angle_increment;
                current_points.emplace_back(
                    range * cos(angle),
                    range * sin(angle)
                );
            }
        }

        // ICP 알고리즘
        double dx = 0.0, dy = 0.0, dtheta = 0.0;
        const int max_iterations = 10;
        const double convergence_threshold = 0.001;

        for (int iter = 0; iter < max_iterations; iter++)
        {
            // 최근접점 찾기
            std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> correspondences;
            for (const auto& current_point : current_points)
            {
                double min_dist = std::numeric_limits<double>::max();
                Eigen::Vector2d closest_point;

                for (const auto& prev_point : prev_points)
                {
                    double dist = (current_point - prev_point).norm();
                    if (dist < min_dist)
                    {
                        min_dist = dist;
                        closest_point = prev_point;
                    }
                }

                if (min_dist < 0.5)  // 50cm 이내의 대응점만 사용
                {
                    correspondences.push_back({current_point, closest_point});
                }
            }

            if (correspondences.empty())
            {
                break;
            }

            // 중심점 계산
            Eigen::Vector2d center_current(0, 0), center_prev(0, 0);
            for (const auto& corr : correspondences)
            {
                center_current += corr.first;
                center_prev += corr.second;
            }
            center_current /= correspondences.size();
            center_prev /= correspondences.size();

            // SVD를 이용한 회전 및 변환 계산
            Eigen::Matrix2d H = Eigen::Matrix2d::Zero();
            for (const auto& corr : correspondences)
            {
                Eigen::Vector2d centered_current = corr.first - center_current;
                Eigen::Vector2d centered_prev = corr.second - center_prev;
                H += centered_current * centered_prev.transpose();
            }

            Eigen::JacobiSVD<Eigen::Matrix2d> svd(H, Eigen::ComputeFullU | 
                                                    Eigen::ComputeFullV);
            Eigen::Matrix2d R = svd.matrixV() * svd.matrixU().transpose();
            Eigen::Vector2d t = center_prev - R * center_current;

            // 변환 업데이트
            double current_dx = t.x();
            double current_dy = t.y();
            double current_dtheta = std::atan2(R(1,0), R(0,0));

            dx += current_dx;
            dy += current_dy;
            dtheta += current_dtheta;

            // 수렴 체크
            if (std::abs(current_dx) < convergence_threshold && 
                std::abs(current_dy) < convergence_threshold && 
                std::abs(current_dtheta) < convergence_threshold)
            {
                break;
            }
        }

        return {dx, dy, dtheta};
    }

    void publish_odometry(const rclcpp::Time& stamp)
    {
        // TF 변환 발행
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = stamp;
        transform.header.frame_id = "odom";
        transform.child_frame_id = "base_link";
        transform.transform.translation.x = current_x_;
        transform.transform.translation.y = current_y_;
        transform.transform.translation.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, current_theta_);
        transform.transform.rotation.x = q.x();
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        transform.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(transform);

        // 오도메트리 메시지 발행
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = stamp;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";
        odom.pose.pose.position.x = current_x_;
        odom.pose.pose.position.y = current_y_;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = transform.transform.rotation;

        odom_pub_->publish(odom);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    sensor_msgs::msg::LaserScan previous_scan_;
    bool is_first_scan_;
    double current_x_;
    double current_y_;
    double current_theta_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SlamOdometry>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
