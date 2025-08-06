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

    // Quaternion to yaw
    double qx = msg->pose.pose.orientation.x;
    double qy = msg->pose.pose.orientation.y;
    double qz = msg->pose.pose.orientation.z;
    double qw = msg->pose.pose.orientation.w;
    double siny_cosp = 2.0 * (qw * qz + qx * qy);
    double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    double robot_yaw = std::atan2(siny_cosp, cosy_cosp);

    // Select target waypoint
    Waypoint target = waypoints_[current_wp_index_];
    double dx = target.x - robot_x;
    double dy = target.y - robot_y;
    double distance_to_wp = std::sqrt(dx * dx + dy * dy);

    if (distance_to_wp < lookahead_distance_) {
      current_wp_index_++;
      if (current_wp_index_ >= waypoints_.size()) {
        RCLCPP_INFO(get_logger(), "Final waypoint reached.");
        publishStop();
        return;
      }
      target = waypoints_[current_wp_index_];
      dx = target.x - robot_x;
      dy = target.y - robot_y;
      distance_to_wp = std::sqrt(dx * dx + dy * dy);
    }

    double target_angle = std::atan2(dy, dx);
    double alpha = target_angle - robot_yaw;
    while (alpha > M_PI) alpha -= 2 * M_PI;
    while (alpha < -M_PI) alpha += 2 * M_PI;

    double Ld = std::max(distance_to_wp, lookahead_distance_);
    double curvature = 0.0;
    if (Ld > 0.001) {
      curvature = 2.0 * std::sin(alpha) / Ld;
    }

    double linear_vel = target_speed_;
    double angular_vel = curvature * linear_vel;

    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = this->get_clock()->now();
    cmd.header.frame_id = "base_link";
    cmd.twist.linear.x = linear_vel;
    cmd.twist.angular.z = angular_vel;
    cmd_pub_->publish(cmd);
  }

  void publishStop()
  {
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = this->get_clock()->now();
    cmd.header.frame_id = "base_link";
    cmd.twist.linear.x = 0.0;
    cmd.twist.angular.z = 0.0;
    cmd_pub_->publish(cmd);
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PurePursuitNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
