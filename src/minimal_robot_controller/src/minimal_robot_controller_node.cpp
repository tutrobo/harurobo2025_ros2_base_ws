#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <cobs_bridge_msgs/msg/cobs_bridge_message.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>

// id 1: /cmd_vel
// id 2: /current_pose

#pragma pack(1)
struct Vector2 {
  float x;
  float y;
};

struct Quaternion {
  float x;
  float y;
  float z;
  float w;
};

struct Twist {
  Vector2 linear;
  float z; // angular
};

struct Pose {
  Vector2 position;
  Quaternion orientation;
};
#pragma pack()

class MinimalRobotControllerNode : public rclcpp::Node {
public:
  MinimalRobotControllerNode() : Node("minimal_robot_controller_node") {
    tx_pub_ = this->create_publisher<cobs_bridge_msgs::msg::COBSBridgeMessage>("cobs_bridge_node/tx", 10);
    current_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("current_pose", 10);
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    rx_sub_ = this->create_subscription<cobs_bridge_msgs::msg::COBSBridgeMessage>(
        "cobs_bridge_node/rx", 10, std::bind(&MinimalRobotControllerNode::rx_sub_cb, this, std::placeholders::_1));
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&MinimalRobotControllerNode::cmd_vel_sub_cb, this, std::placeholders::_1));
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10, std::bind(&MinimalRobotControllerNode::joy_sub_cb, this, std::placeholders::_1));
  }

private:
  rclcpp::Publisher<cobs_bridge_msgs::msg::COBSBridgeMessage>::SharedPtr tx_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  rclcpp::Subscription<cobs_bridge_msgs::msg::COBSBridgeMessage>::SharedPtr rx_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

  void rx_sub_cb(const cobs_bridge_msgs::msg::COBSBridgeMessage &msg) {
    if (msg.id == 2 && msg.data.size() == sizeof(Pose)) {
      Pose pose;
      std::copy(msg.data.begin(), msg.data.end(), reinterpret_cast<uint8_t *>(&pose));

      geometry_msgs::msg::PoseStamped pose_stamped;
      pose_stamped.header.stamp = get_clock()->now();
      pose_stamped.header.frame_id = "map";
      pose_stamped.pose.position.x = pose.position.x;
      pose_stamped.pose.position.y = pose.position.y;
      pose_stamped.pose.orientation.x = pose.orientation.x;
      pose_stamped.pose.orientation.y = pose.orientation.y;
      pose_stamped.pose.orientation.z = pose.orientation.z;
      pose_stamped.pose.orientation.w = pose.orientation.w;
      current_pose_pub_->publish(pose_stamped);
    }
  }

  void cmd_vel_sub_cb(const geometry_msgs::msg::Twist &msg) {
    Twist twist;
    twist.linear.x = msg.linear.x;
    twist.linear.y = msg.linear.y;
    twist.z = msg.angular.z;

    cobs_bridge_msgs::msg::COBSBridgeMessage cobs;
    cobs.id = 1;
    cobs.data.resize(sizeof(Twist));
    std::copy(reinterpret_cast<uint8_t *>(&twist), reinterpret_cast<uint8_t *>(&twist) + sizeof(Twist),
              cobs.data.begin());
    tx_pub_->publish(cobs);
  }

  void joy_sub_cb(const sensor_msgs::msg::Joy &msg) {
    geometry_msgs::msg::Twist twist;
    twist.linear.x = msg.axes[0];
    twist.linear.y = -msg.axes[1];
    twist.angular.z = -msg.axes[2];
    cmd_vel_pub_->publish(twist);
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalRobotControllerNode>());
  rclcpp::shutdown();
  return 0;
}
