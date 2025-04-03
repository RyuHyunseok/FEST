/*
 * LiDAR 필터 노드
 * 
 * 기능:
 * - 자기 감지 제거
 * - 노이즈 필터링
 * - 데이터 안정화
 * 
 * 토픽:
 * - 구독: /scan (원본 라이다 데이터)
 * - 발행: /scan_filtered (필터링된 라이다 데이터)
 */

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <vector>
#include <deque>
#include <cmath>

class LidarFilter : public rclcpp::Node 
{
public:
    LidarFilter() : Node("lidar_filter")
    {
        // 파라미터 초기화
        self_radius_ = 0.4;  // 로봇 반경 (20cm)
        window_size_ = 3;    // 이동 평균 윈도우 크기
        noise_threshold_ = 0.1;  // 노이즈 임계값 (10cm)

        // Publisher
        filtered_scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(
            "/laser_filtered", 10);

        // Subscriber
        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/laser", 10, 
            std::bind(&LidarFilter::scan_callback, this, std::placeholders::_1));

        // 이동 평균을 위한 버퍼 초기화
        previous_scans_.clear();
    }

private:
    /**
     * 라이다 스캔 데이터를 처리하는 콜백 함수
     * 1. 자기 감지 제거
     * 2. 이동 평균 필터 적용
     * 3. 급격한 변화 필터링
     * 4. 필터링된 데이터 발행
     * 
     * @param msg 원본 라이다 스캔 데이터
     */
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        sensor_msgs::msg::LaserScan filtered_scan = *msg;
        std::vector<float> filtered_ranges = msg->ranges;

        // 자기 감지 제거
        remove_self_detection(filtered_ranges, msg->angle_min, msg->angle_increment);

        // 이동 평균 필터 적용
        // apply_moving_average(filtered_ranges, msg->ranges);

        // 급격한 변화 필터링
        // filter_sudden_changes(filtered_ranges);

        // 필터링된 데이터 발행
        filtered_scan.ranges = filtered_ranges;
        filtered_scan_pub_->publish(filtered_scan);
    }

    /**
     * 로봇 자신을 감지하는 근접 데이터를 제거하는 함수
     * self_radius_ 내의 감지된 데이터를 무한대 값으로 설정
     * 
     * @param ranges 라이다 거리 데이터 배열
     * @param angle_min 시작 각도
     * @param angle_increment 각도 증분
     */
    void remove_self_detection(std::vector<float>& ranges, float angle_min, float angle_increment)
    {
        for (size_t i = 0; i < ranges.size(); ++i) {
            float current_angle = angle_min + i * angle_increment;
            float range = ranges[i];

            // 유효한 거리값이고 로봇 반경 내부인 경우 제거
            if (std::isfinite(range) && range < self_radius_) {
                ranges[i] = std::numeric_limits<float>::infinity();
            }
        }
    }

    /**
     * 이동 평균 필터를 적용하여 데이터를 안정화하는 함수
     * window_size_ 만큼의 이전 데이터를 사용하여 평균 계산
     * 
     * @param current_ranges 현재 처리할 거리 데이터 배열
     * @param new_ranges 새로 들어온 거리 데이터 배열
     */
    void apply_moving_average(std::vector<float>& current_ranges, const std::vector<float>& new_ranges)
    {
        // 이전 스캔 데이터 저장
        previous_scans_.push_back(new_ranges);
        if (previous_scans_.size() > window_size_) {
            previous_scans_.pop_front();
        }

        // 이동 평균 계산
        if (previous_scans_.size() > 1) {
            for (size_t i = 0; i < current_ranges.size(); ++i) {
                float sum = 0.0f;
                int valid_count = 0;

                for (const auto& scan : previous_scans_) {
                    if (std::isfinite(scan[i])) {
                        sum += scan[i];
                        valid_count++;
                    }
                }

                if (valid_count > 0) {
                    current_ranges[i] = sum / valid_count;
                }
            }
        }
    }

    /**
     * 급격한 거리 변화를 감지하고 보정하는 함수
     * 이웃한 데이터와의 차이가 noise_threshold_보다 큰 경우
     * 주변 값들의 평균으로 보정
     * 
     * @param ranges 보정할 거리 데이터 배열
     */
    void filter_sudden_changes(std::vector<float>& ranges)
    {
        std::vector<float> smoothed_ranges = ranges;

        for (size_t i = 1; i < ranges.size() - 1; ++i) {
            if (std::isfinite(ranges[i])) {
                float prev = std::isfinite(ranges[i-1]) ? ranges[i-1] : ranges[i];
                float next = std::isfinite(ranges[i+1]) ? ranges[i+1] : ranges[i];

                // 이웃한 값들과의 차이가 임계값보다 큰 경우 보정
                if (std::abs(ranges[i] - prev) > noise_threshold_ &&
                    std::abs(ranges[i] - next) > noise_threshold_) {
                    smoothed_ranges[i] = (prev + next) / 2.0f;
                }
            }
        }

        ranges = smoothed_ranges;
    }

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr filtered_scan_pub_;

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

    // Parameters
    double self_radius_;         // 로봇 자기 감지 제거를 위한 반경
    int window_size_;            // 이동 평균 윈도우 크기
    double noise_threshold_;     // 급격한 변화 감지를 위한 임계값

    // 이동 평균을 위한 이전 스캔 데이터 저장
    std::deque<std::vector<float>> previous_scans_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarFilter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
