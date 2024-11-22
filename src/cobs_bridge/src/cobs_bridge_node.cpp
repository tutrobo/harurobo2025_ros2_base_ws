#include <deque>
#include <functional>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <cobs_bridge_interfaces/msg/cobs_bridge_message.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

#include "cobs.h"

class COBSBridgeNode : public rclcpp::Node {
public:
  COBSBridgeNode() : Node("cobs_bridge") {
    rx_pub_ =
        this->create_publisher<cobs_bridge_interfaces::msg::COBSBridgeMessage>(
            "~/rx", rclcpp::QoS(10).best_effort());
    serial_write_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(
        "serial_write", rclcpp::QoS(10).best_effort());

    tx_sub_ = this->create_subscription<
        cobs_bridge_interfaces::msg::COBSBridgeMessage>(
        "~/tx", rclcpp::QoS(10).best_effort(),
        std::bind(&COBSBridgeNode::tx_sub_cb, this, std::placeholders::_1));
    serial_read_sub_ =
        this->create_subscription<std_msgs::msg::UInt8MultiArray>(
            "serial_read", rclcpp::QoS(10).best_effort(),
            std::bind(&COBSBridgeNode::serial_read_sub_cb, this,
                      std::placeholders::_1));
  }

private:
  std::deque<uint8_t> serial_read_queue_;
  std::vector<uint8_t> rx_buf_;

  rclcpp::Publisher<cobs_bridge_interfaces::msg::COBSBridgeMessage>::SharedPtr
      rx_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr
      serial_write_pub_;

  rclcpp::Subscription<
      cobs_bridge_interfaces::msg::COBSBridgeMessage>::SharedPtr tx_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr
      serial_read_sub_;

  void tx_sub_cb(
      const cobs_bridge_interfaces::msg::COBSBridgeMessage::SharedPtr msg) {
    std_msgs::msg::UInt8MultiArray tx_msg;

    // id(1 byte) + data(n byte) + checksum(1 byte) + delimiter(1 byte)
    if (!msg->data.empty()) {
      tx_msg.data.resize(COBS_ENCODE_DST_BUF_LEN_MAX(msg->data.size()) + 3);
      cobs_encode_result res =
          cobs_encode(tx_msg.data.data() + 1, tx_msg.data.size() - 3,
                      msg->data.data(), msg->data.size());
      if (res.status != COBS_ENCODE_OK) {
        return;
      }
      tx_msg.data.resize(res.out_len + 3);
    } else {
      tx_msg.data.resize(3);
    }

    tx_msg.data[0] = msg->id;
    tx_msg.data[tx_msg.data.size() - 2] =
        checksum(tx_msg.data.data(), tx_msg.data.size() - 2);
    tx_msg.data[tx_msg.data.size() - 1] = 0; // delimiter
    serial_write_pub_->publish(tx_msg);
  }

  void serial_read_sub_cb(const std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
    for (const auto &e : msg->data) {
      serial_read_queue_.push_back(e);
    }

    while (true) {
      if (!read_to_end()) {
        return;
      }

      if (rx_buf_.size() < 3) {
        rx_buf_.clear();
        return;
      }
      if (checksum(rx_buf_.data(), rx_buf_.size() - 2) !=
          rx_buf_[rx_buf_.size() - 2]) {
        rx_buf_.clear();
        return;
      }

      cobs_bridge_interfaces::msg::COBSBridgeMessage rx_msg;
      if (rx_buf_.size() > 3) {
        rx_msg.data.resize(COBS_DECODE_DST_BUF_LEN_MAX(rx_buf_.size() - 3));
        cobs_decode_result res =
            cobs_decode(rx_msg.data.data(), rx_msg.data.size(),
                        rx_buf_.data() + 1, rx_buf_.size() - 3);
        if (res.status != COBS_DECODE_OK) {
          rx_buf_.clear();
          return;
        }
        rx_msg.data.resize(res.out_len);
      } else {
        rx_msg.data.resize(0);
      }

      rx_msg.id = rx_buf_[0];
      rx_buf_.clear();
      rx_pub_->publish(rx_msg);
    }
  }

  bool read_to_end() {
    while (!serial_read_queue_.empty()) {
      uint8_t tmp = serial_read_queue_.front();
      serial_read_queue_.pop_front();
      rx_buf_.push_back(tmp);
      if (tmp == 0x00) {
        return true;
      }
    }
    return false;
  }

  uint8_t checksum(const uint8_t *data, size_t size) {
    uint8_t res = 0;
    for (size_t i = 0; i < size; ++i) {
      res += data[i];
    }
    return 0x80 | (res & 0x7F);
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<COBSBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
