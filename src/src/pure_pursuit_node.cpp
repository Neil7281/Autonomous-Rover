#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <cmath>
#include <vector>

class PurePursuitNode : public rclcpp::Node
{
public:
  PurePursuitNode() : Node("pure_pursuit_node")
  {
    this->declare_parameter<double>("lookahead_distance", 1.0);
    this->declare_parameter<double>("target_speed", 0.5);
    lookahead_distance_ = this->get_parameter("lookahead_distance").as_double();
    target_speed_ = this->get_parameter("target_speed").as_double();

    // Subscribers
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odometry/filtered", 10,
      std::bind(&PurePursuitNode::odomCallback, this, std::placeholders::_1));

    waypoints_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/waypoints", 10,
      std::bind(&PurePursuitNode::waypointsCallback, this, std::placeholders::_1));

    // Publisher
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      "/mavros/setpoint_velocity/cmd_vel", 10);

    RCLCPP_INFO(this->get_logger(), "Pure Pursuit node initialized.");
  }

private:
  struct Waypoint { double x; double y; };
  std::vector<Waypoint> waypoints_;
  size_t current_wp_index_ = 0;
  double lookahead_distance_;
  double target_speed_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr waypoints_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;

  void waypointsCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    const std::vector<double>& wp_flat = msg->data;
    if (wp_flat.size() % 2 != 0) {
      RCLCPP_WARN(this->get_logger(), "Waypoint list must contain pairs of (x, y). Ignored.");
      return;
    }

    waypoints_.clear();
    for (size_t i = 0; i + 1 < wp_flat.size(); i += 2) {
      Waypoint wp{wp_flat[i], wp_flat[i + 1]};
      waypoints_.push_back(wp);
    }

    current_wp_index_ = 0;
    RCLCPP_INFO(this->get_logger(), "Received %ld waypoints.", waypoints_.size());
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (waypoints_.empty() || current_wp_index_ >= waypoints_.size()) {
      publishStop();
      return;
    }

    double robot_x = msg->pose.pose.position.x;
    double robot_y = msg->pose.pose.position.y;

    // Qua
