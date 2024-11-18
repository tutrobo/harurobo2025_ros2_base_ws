#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cobs_bridge_msgs/msg/cobs_bridge_message.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_srvs/srv/trigger.hpp>

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
    sub_cb_group_ =
        create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    srv_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    // pub
    tx_pub_ = create_publisher<cobs_bridge_msgs::msg::COBSBridgeMessage>(
        "cobs_bridge_node/tx", rclcpp::QoS(10).best_effort());
    current_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
        "current_pose", rclcpp::QoS(10).best_effort());
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(
        "cmd_vel", rclcpp::QoS(10).best_effort());

    // sub
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = sub_cb_group_;
    rx_sub_ = create_subscription<cobs_bridge_msgs::msg::COBSBridgeMessage>(
        "cobs_bridge_node/rx", rclcpp::QoS(10).best_effort(),
        std::bind(&MinimalRobotControllerNode::rx_sub_cb, this,
                  std::placeholders::_1),
        sub_options);
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", rclcpp::QoS(10).best_effort(),
        std::bind(&MinimalRobotControllerNode::cmd_vel_sub_cb, this,
                  std::placeholders::_1),
        sub_options);
    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
        "joy", rclcpp::QoS(10),
        std::bind(&MinimalRobotControllerNode::joy_sub_cb, this,
                  std::placeholders::_1),
        sub_options);

    // service
    reset_pose_srv_ = create_service<std_srvs::srv::Trigger>(
        "reset_pose",
        std::bind(&MinimalRobotControllerNode::reset_pose_srv_cb, this,
                  std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, srv_cb_group_);
  }

private:
  rclcpp::CallbackGroup::SharedPtr sub_cb_group_;
  rclcpp::CallbackGroup::SharedPtr srv_cb_group_;

  rclcpp::Publisher<cobs_bridge_msgs::msg::COBSBridgeMessage>::SharedPtr
      tx_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
      current_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  rclcpp::Subscription<cobs_bridge_msgs::msg::COBSBridgeMessage>::SharedPtr
      rx_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_pose_srv_;
  std::mutex reset_pose_mtx_;
  std::optional<std::promise<bool>> reset_pose_res_;

  std::mutex quat_mtx_;
  tf2::Quaternion current_quat_;
  std::optional<tf2::Quaternion> offset_quat_;

  void rx_sub_cb(
      const std::shared_ptr<cobs_bridge_msgs::msg::COBSBridgeMessage> msg) {
    if (msg->id == 2 && msg->data.size() == sizeof(Pose)) {
      auto pose = reinterpret_cast<const Pose *>(msg->data.data());
      geometry_msgs::msg::PoseStamped current_pose_msg;
      current_pose_msg.header.stamp = get_clock()->now();
      current_pose_msg.header.frame_id = "map";
      current_pose_msg.pose.position.x = pose->position.x;
      current_pose_msg.pose.position.y = pose->position.y;
      current_pose_msg.pose.orientation.x = pose->orientation.x;
      current_pose_msg.pose.orientation.y = pose->orientation.y;
      current_pose_msg.pose.orientation.z = pose->orientation.z;
      current_pose_msg.pose.orientation.w = pose->orientation.w;
      {
        std::lock_guard lock_{quat_mtx_};
        tf2::fromMsg(current_pose_msg.pose.orientation, current_quat_);
        if (offset_quat_) {
          current_pose_msg.pose.orientation =
              tf2::toMsg(current_quat_ * offset_quat_->inverse());
        }
      }
      current_pose_pub_->publish(current_pose_msg);
    } else if (msg->id == 3 && msg->data.size() == 0) {
      std::lock_guard lock_{reset_pose_mtx_};
      if (reset_pose_res_) {
        reset_pose_res_->set_value(true);
        reset_pose_res_ = std::nullopt;
      }
    }
  }

  void cmd_vel_sub_cb(const std::shared_ptr<geometry_msgs::msg::Twist> msg) {
    Twist twist;
    twist.linear.x = msg->linear.x;
    twist.linear.y = msg->linear.y;
    twist.z = msg->angular.z;

    cobs_bridge_msgs::msg::COBSBridgeMessage tx_msg;
    tx_msg.id = 1;
    tx_msg.data.resize(sizeof(Twist));
    std::copy(reinterpret_cast<uint8_t *>(&twist),
              reinterpret_cast<uint8_t *>(&twist) + sizeof(Twist),
              tx_msg.data.begin());
    tx_pub_->publish(tx_msg);
  }

  void joy_sub_cb(const std::shared_ptr<sensor_msgs::msg::Joy> msg) {
    geometry_msgs::msg::Twist twist;
    twist.linear.x = msg->axes[0];
    twist.linear.y = -msg->axes[1];
    twist.angular.z = -0.5 * msg->axes[2];
    cmd_vel_pub_->publish(twist);
  }

  void reset_pose_srv_cb(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    std::future<bool> f;
    {
      std::lock_guard lock_{reset_pose_mtx_};
      std::promise<bool> p;
      f = std::move(p.get_future());
      reset_pose_res_ = std::move(p);
    }
    cobs_bridge_msgs::msg::COBSBridgeMessage tx_msg;
    tx_msg.id = 3;
    tx_pub_->publish(tx_msg);
    std::future_status result = f.wait_for(std::chrono::seconds(3));
    if (result != std::future_status::timeout) {
      {
        std::lock_guard lock_{quat_mtx_};
        offset_quat_ = current_quat_;
      }
      response->success = f.get();
    } else {
      response->success = false;
    }
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MinimalRobotControllerNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
