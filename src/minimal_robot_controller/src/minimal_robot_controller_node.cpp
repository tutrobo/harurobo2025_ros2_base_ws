#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <cobs_bridge_msgs/msg/cobs_bridge_message.hpp>
#include <example_interfaces/srv/trigger.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>

// id 1: /cmd_vel
// id 2: /current_pose
// id 3: /reset_pose

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
    // pub
    tx_pub_ = this->create_publisher<cobs_bridge_msgs::msg::COBSBridgeMessage>(
        "cobs_bridge_node/tx", rclcpp::QoS(10).best_effort());
    current_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "current_pose", rclcpp::QoS(10).best_effort());
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "cmd_vel", rclcpp::QoS(10).best_effort());

    // sub
    rx_sub_ =
        this->create_subscription<cobs_bridge_msgs::msg::COBSBridgeMessage>(
            "cobs_bridge_node/rx", rclcpp::QoS(10).best_effort(),
            std::bind(&MinimalRobotControllerNode::rx_sub_cb, this,
                      std::placeholders::_1));
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", rclcpp::QoS(10).best_effort(),
        std::bind(&MinimalRobotControllerNode::cmd_vel_sub_cb, this,
                  std::placeholders::_1));
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", rclcpp::QoS(10),
        std::bind(&MinimalRobotControllerNode::joy_sub_cb, this,
                  std::placeholders::_1));

    // service
    reset_pose_srv_ = this->create_service<example_interfaces::srv::Trigger>(
        "reset_pose",
        std::bind(&MinimalRobotControllerNode::reset_pose_srv_cb, this,
                  std::placeholders::_1, std::placeholders::_2));
  }

private:
  rclcpp::Publisher<cobs_bridge_msgs::msg::COBSBridgeMessage>::SharedPtr
      tx_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
      current_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  rclcpp::Subscription<cobs_bridge_msgs::msg::COBSBridgeMessage>::SharedPtr
      rx_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

  rclcpp::Service<example_interfaces::srv::Trigger>::SharedPtr reset_pose_srv_;
  std::mutex reset_pose_mtx_;
  std::optional<std::promise<bool>> reset_pose_result_;

  void rx_sub_cb(
      const std::shared_ptr<cobs_bridge_msgs::msg::COBSBridgeMessage> msg) {
    if (msg->id == 2 && msg->data.size() == sizeof(Pose)) {
      auto pose = reinterpret_cast<const Pose *>(msg->data.data());
      geometry_msgs::msg::PoseStamped pose_stamped;
      pose_stamped.header.stamp = get_clock()->now();
      pose_stamped.header.frame_id = "map";
      pose_stamped.pose.position.x = pose->position.x;
      pose_stamped.pose.position.y = pose->position.y;
      pose_stamped.pose.orientation.x = pose->orientation.x;
      pose_stamped.pose.orientation.y = pose->orientation.y;
      pose_stamped.pose.orientation.z = pose->orientation.z;
      pose_stamped.pose.orientation.w = pose->orientation.w;
      current_pose_pub_->publish(pose_stamped);
    } else if (msg->id == 3 && msg->data.size() == 0) {
      std::lock_guard lock_{reset_pose_mtx_};
      if (reset_pose_result_) {
        reset_pose_result_->set_value(true);
        reset_pose_result_ = std::nullopt;
      }
    }
  }

  void cmd_vel_sub_cb(const std::shared_ptr<geometry_msgs::msg::Twist> msg) {
    Twist twist;
    twist.linear.x = msg->linear.x;
    twist.linear.y = msg->linear.y;
    twist.z = msg->angular.z;

    cobs_bridge_msgs::msg::COBSBridgeMessage cobs;
    cobs.id = 1;
    cobs.data.resize(sizeof(Twist));
    std::copy(reinterpret_cast<uint8_t *>(&twist),
              reinterpret_cast<uint8_t *>(&twist) + sizeof(Twist),
              cobs.data.begin());
    tx_pub_->publish(cobs);
  }

  void joy_sub_cb(const std::shared_ptr<sensor_msgs::msg::Joy> msg) {
    geometry_msgs::msg::Twist twist;
    twist.linear.x = msg->axes[0];
    twist.linear.y = -msg->axes[1];
    twist.angular.z = -0.5 * msg->axes[2];
    cmd_vel_pub_->publish(twist);
  }

  void reset_pose_srv_cb(
      const std::shared_ptr<example_interfaces::srv::Trigger::Request> request,
      std::shared_ptr<example_interfaces::srv::Trigger::Response> response) {
    std::future<bool> f;
    {
      std::lock_guard lock_{reset_pose_mtx_};
      std::promise<bool> p;
      f = std::move(p.get_future());
      reset_pose_result_ = std::optional{std::move(p)};
    }
    response->success = f.get();
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalRobotControllerNode>());
  rclcpp::shutdown();
  return 0;
}
