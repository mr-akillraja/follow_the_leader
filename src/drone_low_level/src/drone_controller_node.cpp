#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <string>
#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>

using std::placeholders::_1;
using namespace std::chrono_literals;

class DroneControllerNode : public rclcpp::Node
{
public:
  DroneControllerNode() : Node("drone_controller_node")
  {
    // Declare parameters
    this->declare_parameter<std::string>("drone_name", "drone1");
    this->declare_parameter<double>("alpha", 0.1); // Smoothing factor (0 = no change, 1 = instant change)

    std::string drone_name;
    this->get_parameter("drone_name", drone_name);
    this->get_parameter("alpha", alpha_);

    std::string sub_topic = "/" + drone_name + "/cmd_vel_pso";
    std::string pub_topic = "/" + drone_name + "/cmd_vel";

    RCLCPP_INFO(this->get_logger(), "Starting controller for %s. Sub: %s, Pub: %s", 
                drone_name.c_str(), sub_topic.c_str(), pub_topic.c_str());

    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      sub_topic, 10, std::bind(&DroneControllerNode::target_velocity_callback, this, _1));

    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(pub_topic, 10);

    // Initialize velocities
    current_vel_ = geometry_msgs::msg::Twist();
    target_vel_ = geometry_msgs::msg::Twist();

    // Loop at 50Hz
    timer_ = this->create_wall_timer(
      20ms, std::bind(&DroneControllerNode::control_loop, this));
  }

private:
  void target_velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    target_vel_ = *msg;
  }

  void control_loop()
  {
    // Smoothly interpolate current velocity to target velocity (Exponential Moving Average)
    current_vel_.linear.x += alpha_ * (target_vel_.linear.x - current_vel_.linear.x);
    current_vel_.linear.y += alpha_ * (target_vel_.linear.y - current_vel_.linear.y);
    current_vel_.linear.z += alpha_ * (target_vel_.linear.z - current_vel_.linear.z);
    
    current_vel_.angular.z += alpha_ * (target_vel_.angular.z - current_vel_.angular.z);

    // Publish the smoothed velocity to Gazebo plugin
    publisher_->publish(current_vel_);
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  geometry_msgs::msg::Twist current_vel_;
  geometry_msgs::msg::Twist target_vel_;
  double alpha_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DroneControllerNode>());
  rclcpp::shutdown();
  return 0;
}
