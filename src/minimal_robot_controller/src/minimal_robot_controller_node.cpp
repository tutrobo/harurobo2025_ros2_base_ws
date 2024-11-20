#include <chrono>
#include <functional>

#include <rclcpp/rclcpp.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cobs_bridge_interfaces/msg/cobs_bridge_message.hpp>
#include <cobs_bridge_interfaces/srv/cobs_bridge_service.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
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
  MinimalRobotControllerNode() : Node("minimal_robot_controller") {
    srv_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    // pub
    tx_pub_ = create_publisher<cobs_bridge_interfaces::msg::COBSBridgeMessage>(
        "cobs_bridge/tx", rclcpp::QoS(10).best_effort());
    current_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
        "current_pose", rclcpp::QoS(10).best_effort());
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(
        "cmd_vel", rclcpp::QoS(10).best_effort());

    // sub
    rx_sub_ =
        create_subscription<cobs_bridge_interfaces::msg::COBSBridgeMessage>(
            "cobs_bridge/rx", rclcpp::QoS(10).best_effort(),
            std::bind(&MinimalRobotControllerNode::rx_sub_cb, this,
                      std::placeholders::_1));
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", rclcpp::QoS(10).best_effort(),
        std::bind(&MinimalRobotControllerNode::cmd_vel_sub_cb, this,
                  std::placeholders::_1));
    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
        "joy", rclcpp::QoS(10),
        std::bind(&MinimalRobotControllerNode::joy_sub_cb, this,
                  std::placeholders::_1));

    // service
    reset_pose_srv_ = create_service<std_srvs::srv::Trigger>(
        "reset_pose",
        std::bind(&MinimalRobotControllerNode::reset_pose_srv_cb, this,
                  std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, srv_cb_group_);
    call_client_ =
        create_client<cobs_bridge_interfaces::srv::COBSBridgeService>(
            "cobs_bridge_service/call", rmw_qos_profile_services_default,
            srv_cb_group_);
  }

private:
  rclcpp::CallbackGroup::SharedPtr srv_cb_group_;

  rclcpp::Publisher<cobs_bridge_interfaces::msg::COBSBridgeMessage>::SharedPtr
      tx_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
      current_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  rclcpp::Subscription<
      cobs_bridge_interfaces::msg::COBSBridgeMessage>::SharedPtr rx_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_pose_srv_;
  rclcpp::Client<cobs_bridge_interfaces::srv::COBSBridgeService>::SharedPtr
      call_client_;

  void rx_sub_cb(
      const cobs_bridge_interfaces::msg::COBSBridgeMessage::SharedPtr msg) {
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
      current_pose_pub_->publish(current_pose_msg);
    }
  }

  void cmd_vel_sub_cb(const geometry_msgs::msg::Twist::SharedPtr msg) {
    Twist twist;
    twist.linear.x = msg->linear.x;
    twist.linear.y = msg->linear.y;
    twist.z = msg->angular.z;

    cobs_bridge_interfaces::msg::COBSBridgeMessage tx_msg;
    tx_msg.id = 1;
    tx_msg.data.resize(sizeof(Twist));
    std::copy(reinterpret_cast<uint8_t *>(&twist),
              reinterpret_cast<uint8_t *>(&twist) + sizeof(Twist),
              tx_msg.data.begin());
    tx_pub_->publish(tx_msg);
  }

  void joy_sub_cb(const sensor_msgs::msg::Joy::SharedPtr msg) {
    geometry_msgs::msg::Twist twist;
    twist.linear.x = msg->axes[0];
    twist.linear.y = -msg->axes[1];
    twist.angular.z = -2 * M_PI * msg->axes[2];
    cmd_vel_pub_->publish(twist);
  }

  void
  reset_pose_srv_cb(const std_srvs::srv::Trigger::Request::SharedPtr request,
                    std_srvs::srv::Trigger::Response::SharedPtr response) {
    auto call_request = std::make_shared<
        cobs_bridge_interfaces::srv::COBSBridgeService::Request>();
    call_request->id = 3;

    auto result_future = call_client_->async_send_request(call_request);
    std::future_status status = result_future.wait_for(std::chrono::seconds(3));
    if (status == std::future_status::ready) {
      auto call_response = result_future.get();
      response->success = call_response->success;
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
