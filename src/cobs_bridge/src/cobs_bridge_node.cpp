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
  std::vector<uint8_t> buf_;

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
    std::vector<uint8_t> src(msg->data.size() + 1);
    src[0] = msg->id;
    std::copy(msg->data.begin(), msg->data.end(), src.begin() + 1);
    src.push_back(checksum(src.data(), src.size()));

    std::vector<uint8_t> dest;
    if (!cobs_encode(src, tx_msg.data)) {
      return;
    }

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

      std::vector<uint8_t> dest;
      if (!cobs_decode(buf_, dest)) {
        buf_.clear();
        continue;
      }

      if (checksum(dest.data(), dest.size() - 1) != dest[dest.size()]) {
        buf_.clear();
        continue;
      }

      cobs_bridge_interfaces::msg::COBSBridgeMessage rx_msg;
      rx_msg.id = dest[0];
      rx_msg.data.resize(dest.size() - 1);
      std::copy(dest.begin() + 1, dest.end(), rx_msg.data.begin());
      buf_.clear();
      rx_pub_->publish(rx_msg);
    }
  }

  bool read_to_end() {
    while (!serial_read_queue_.empty()) {
      uint8_t tmp = serial_read_queue_.front();
      serial_read_queue_.pop_front();
      buf_.push_back(tmp);
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

  bool cobs_encode(const std::vector<uint8_t> &src,
                   std::vector<uint8_t> &dest) {
    dest.resize(COBS_ENCODE_DST_BUF_LEN_MAX(src.size()));
    cobs_encode_result res =
        ::cobs_encode(dest.data(), dest.size(), src.data(), src.size());
    if (res.status != COBS_ENCODE_OK) {
      return false;
    }
    dest.resize(res.out_len + 1);
    dest[res.out_len] = 0;
    return true;
  }

  bool cobs_decode(const std::vector<uint8_t> &src,
                   std::vector<uint8_t> &dest) {
    dest.resize(COBS_DECODE_DST_BUF_LEN_MAX(src.size()));
    cobs_decode_result res =
        ::cobs_decode(dest.data(), dest.size(), src.data(), src.size());
    if (res.status != COBS_DECODE_OK &&
        res.status != COBS_DECODE_ZERO_BYTE_IN_INPUT) {
      return false;
    }
    dest.resize(res.out_len);
    return true;
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<COBSBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
