#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cmath>
#include <vector>

class PurePursuitNode : public rclcpp::Node
{
public:
  PurePursuitNode() : Node("pure_pursuit_node")
  {
    // Declare and get parameters
    this->declare_parameter<double>("lookahead_distance", 1.0);
    this->declare_parameter<double>("target_speed", 0.5);
    this->declare_parameter<std::vector<double>>("waypoints", std::vector<double>{});
    lookahead_distance_ = this->get_parameter("lookahead_distance").as_double();
    target_speed_ = this->get_parameter("target_speed").as_double();
    std::vector<double> wp_flat = this->get_parameter("waypoints").as_double_array();
    // Convert flat list [x1,y1,x2,y2,...] into pair list
    for (size_t i = 0; i + 1 < wp_flat.size(); i += 2) {
      Waypoint wp;
      wp.x = wp_flat[i];
      wp.y = wp_flat[i+1];
      waypoints_.push_back(wp);
    }
    if (waypoints_.empty()) {
      RCLCPP_ERROR(get_logger(), "No waypoints provided! Please set the 'waypoints' parameter.");
    }

    // Create subscriber to odometry (fused localization)
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filtered", 10, std::bind(&PurePursuitNode::odomCallback, this, std::placeholders::_1));
    // Create publisher for velocity commands to MAVROS (geometry_msgs::TwistStamped)
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
        "/mavros/setpoint_velocity/cmd_vel", 10);
  }

private:
  struct Waypoint { double x; double y; };
  std::vector<Waypoint> waypoints_;
  size_t current_wp_index_ = 0;
  double lookahead_distance_;
  double target_speed_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (waypoints_.empty() || current_wp_index_ >= waypoints_.size()) {
      // No more waypoints or none provided
      publishStop();
      return;
    }
    // Current robot pose (from odometry in ENU map/odom frame)
    double robot_x = msg->pose.pose.position.x;
    double robot_y = msg->pose.pose.position.y;
    // Orientation (yaw) from quaternion
    double qx = msg->pose.pose.orientation.x;
    double qy = msg->pose.pose.orientation.y;
    double qz = msg->pose.pose.orientation.z;
    double qw = msg->pose.pose.orientation.w;
    // Convert quaternion to yaw (assuming orientation covariance is acceptable)
    double siny_cosp = 2.0 * (qw * qz + qx * qy);
    double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    double robot_yaw = std::atan2(siny_cosp, cosy_cosp);

    // Lookahead target: choose the current waypoint as target, or a point along segment if needed
    Waypoint target = waypoints_[current_wp_index_];
    double dx = target.x - robot_x;
    double dy = target.y - robot_y;
    double distance_to_wp = std::sqrt(dx*dx + dy*dy);
    // If close to current waypoint, move to next waypoint
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
      distance_to_wp = std::sqrt(dx*dx + dy*dy);
    }
    // Transform target point to vehicle coordinates (relative to robot base_link)
    double target_angle = std::atan2(dy, dx);               // angle of target in global frame
    double alpha = target_angle - robot_yaw;                // angle to target relative to robot heading
    // Normalize alpha to [-pi, pi]
    while (alpha > M_PI) alpha -= 2*M_PI;
    while (alpha < -M_PI) alpha += 2*M_PI;
    // Compute curvature = 2 * sin(alpha) / Ld, where Ld = lookahead_distance or distance_to_wp
    double Ld = std::max(distance_to_wp, lookahead_distance_);
    double curvature = 0.0;
    if (Ld > 0.001) {
      curvature = 2.0 * std::sin(alpha) / Ld;
    }
    // Compute target angular velocity (yaw rate) = curvature * linear velocity
    double linear_vel = target_speed_;
    double angular_vel = curvature * linear_vel;
    // Limit angular velocity for stability (optional)
    // double max_yaw_rate = 1.0; // [rad/s], for example
    // angular_vel = std::clamp(angular_vel, -max_yaw_rate, max_yaw_rate);

    // Publish Twist command
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = this->get_clock()->now();
    cmd.header.frame_id = "base_link";  // velocity in body frame
    cmd.twist.linear.x = linear_vel;
    cmd.twist.linear.y = 0.0;
    cmd.twist.linear.z = 0.0;
    cmd.twist.angular.z = angular_vel;
    cmd.twist.angular.x = 0.0;
    cmd.twist.angular.y = 0.0;
    cmd_pub_->publish(cmd);
  }

  void publishStop()
  {
    // Publish zero velocity to stop the rover
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
  // For offboard control, PX4 requires sending a few setpoints before entering OFFBOARD mode
  RCLCPP_INFO(node->get_logger(), "Pure Pursuit node started. Waiting for odometry and publishing setpoints...");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
