#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <vector>

class WaypointPublisherNode : public rclcpp::Node
{
public:
  WaypointPublisherNode() : Node("waypoint_publisher_node")
  {
    this->declare_parameter<std::vector<double>>("waypoints", std::vector<double>{0.0, 0.0, 5.0, 0.0, 5.0, 5.0});
    waypoints_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/waypoints", 10);
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&WaypointPublisherNode::publishWaypoints, this));
    RCLCPP_INFO(this->get_logger(), "Waypoint Publisher Node initialized.");
  }

private:
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr waypoints_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  void publishWaypoints()
  {
    std_msgs::msg::Float64MultiArray msg;
    msg.data = this->get_parameter("waypoints").as_double_array();
    waypoints_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Published %ld waypoint values.", msg.data.size());
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WaypointPublisherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
