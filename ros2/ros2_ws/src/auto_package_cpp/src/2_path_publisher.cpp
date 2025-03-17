#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include <fstream>
#include <string>
#include <vector>
#include <cmath>
#include <memory>
#include <cstdint>
#include <iso646.h> // Alternative operator spellings
#include <sstream>  // istringstream 을 위해 필요

using namespace std::chrono_literals;  // 20ms 사용을 위해 필요

class PathPub : public rclcpp::Node
{
public:
  PathPub() : Node("path_pub"), count_(0), is_odom_(false)
  {
    // Create publishers
    global_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("global_path", 10);
    
    // Create subscription
    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&PathPub::listener_callback, this, std::placeholders::_1));
    
    // Initialize global path message
    global_path_msg_.header.frame_id = "map";
    
    // Read path from file
    std::ifstream file("C:\\Users\\SSAFY\\Desktop\\catkin_ws\\src\\auto_package_cpp\\path\\test.txt");
    if (not file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open path file");
      return;
    }
    
    std::string line;
    while (std::getline(file, line)) {
      std::istringstream iss(line);
      double x, y;
      if (not (iss >> x >> y)) {
        continue; // Skip malformed lines
      }
      
      geometry_msgs::msg::PoseStamped read_pose;
      read_pose.pose.position.x = x;
      read_pose.pose.position.y = y;
      read_pose.pose.orientation.w = 1.0;
      global_path_msg_.poses.push_back(read_pose);
    }
    file.close();
    
    timer_ = this->create_wall_timer(
      20ms, std::bind(&PathPub::timer_callback, this));
  }

private:
  void listener_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    is_odom_ = true;
    odom_msg_ = *msg;
  }
  
  void timer_callback()
  {
    if (is_odom_) {
      // Publish global path every 10 cycles
      if (count_ % 10 == 0) {
        global_path_msg_.header.stamp = this->now();
        global_path_pub_->publish(global_path_msg_);
      }
      count_++;
    }
  }
  
  // Subscribers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  
  // Publishers
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_path_pub_;
  
  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
  
  // Member variables
  nav_msgs::msg::Odometry odom_msg_;
  nav_msgs::msg::Path global_path_msg_;
  bool is_odom_;
  std::uint32_t count_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PathPub>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}