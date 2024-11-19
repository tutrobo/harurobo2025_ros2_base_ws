#include <chrono>
#include <functional>
#include <mutex>
#include <optional>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <cobs_bridge_interfaces/msg/cobs_bridge_message.hpp>
#include <cobs_bridge_interfaces/srv/cobs_bridge_service.hpp>

class COBSBridgeServiceNode : public rclcpp::Node {
public:
  COBSBridgeServiceNode() : Node("cobs_bridge_service") {
    srv_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    tx_pub_ = create_publisher<cobs_bridge_interfaces::msg::COBSBridgeMessage>(
        "cobs_bridge/tx", rclcpp::QoS(10).best_effort());

    rx_sub_ =
        create_subscription<cobs_bridge_interfaces::msg::COBSBridgeMessage>(
            "cobs_bridge/rx", rclcpp::QoS(10).best_effort(),
            std::bind(&COBSBridgeServiceNode::rx_sub_cb, this,
                      std::placeholders::_1));

    call_srv_ = create_service<cobs_bridge_interfaces::srv::COBSBridgeService>(
        "~/call",
        std::bind(&COBSBridgeServiceNode::call_srv_cb, this,
                  std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, srv_cb_group_);
  }

private:
  rclcpp::CallbackGroup::SharedPtr srv_cb_group_;
  rclcpp::Publisher<cobs_bridge_interfaces::msg::COBSBridgeMessage>::SharedPtr
      tx_pub_;
  rclcpp::Subscription<
      cobs_bridge_interfaces::msg::COBSBridgeMessage>::SharedPtr rx_sub_;
  rclcpp::Service<cobs_bridge_interfaces::srv::COBSBridgeService>::SharedPtr
      call_srv_;

  std::mutex call_mtx_;
  std::optional<uint8_t> call_id_;
  std::optional<
      std::promise<cobs_bridge_interfaces::msg::COBSBridgeMessage::SharedPtr>>
      call_res_;

  void rx_sub_cb(
      const cobs_bridge_interfaces::msg::COBSBridgeMessage::SharedPtr msg) {
    std::lock_guard lock{call_mtx_};
    if (call_id_ && call_res_ && msg->id == *call_id_) {
      call_res_->set_value(msg);
      call_id_ = std::nullopt;
      call_res_ = std::nullopt;
    }
  }

  void call_srv_cb(
      const cobs_bridge_interfaces::srv::COBSBridgeService::Request::SharedPtr
          request,
      cobs_bridge_interfaces::srv::COBSBridgeService::Response::SharedPtr
          response) {
    std::future<cobs_bridge_interfaces::msg::COBSBridgeMessage::SharedPtr>
        result_future;
    {
      std::lock_guard lock{call_mtx_};
      std::promise<cobs_bridge_interfaces::msg::COBSBridgeMessage::SharedPtr> p;
      result_future = std::move(p.get_future());
      call_id_ = request->id;
      call_res_ = std::move(p);
    }

    cobs_bridge_interfaces::msg::COBSBridgeMessage tx_msg;
    tx_msg.id = request->id;
    tx_msg.data = std::move(request->data);
    tx_pub_->publish(tx_msg);

    std::future_status status = result_future.wait_for(std::chrono::seconds(3));
    if (status == std::future_status::ready) {
      auto rx_msg = result_future.get();
      response->success = true;
      response->data = std::move(rx_msg->data);
    } else {
      response->success = false;
    }
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<COBSBridgeServiceNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
