#include "ros2_node/motion_state_request_node.h"

#include <glog/logging.h>

#include "data/motion_state_request/motion_state_request_mailbox.h"
#include "variant_store/variant_store.h"

namespace ros2 {

bool MotionStateRequestNode::Init() {
  if (!LogicNode::Init()) {
    return false;
  }

  if (!param_->subscribe_topics.has_value()) {
    LOG(ERROR) << "MotionStateRequestNode: subscribe_topics is missing";
    return false;
  }

  const auto& topics = param_->subscribe_topics.value();
  const auto topic = topics.find("state_request");
  if (topic == topics.end() || topic->second.empty()) {
    LOG(ERROR) << "MotionStateRequestNode: state_request topic is missing";
    return false;
  }

  request_mailbox_ =
      data::VariantStore::GetInstance()
          .GetOrCreate<data::MotionStateRequestMailbox>(
              "motion/state_request_mailbox");

  const auto qos =
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();

  state_request_subscriber_ =
      create_subscription<interface_protocol::msg::MotionStateRequest>(
          topic->second, qos,
          [this](
              const interface_protocol::msg::MotionStateRequest::SharedPtr msg) {
            StateRequestCallback(msg);
          });

  LOG(INFO) << "MotionStateRequestNode subscribed to " << topic->second;
  return true;
}

void MotionStateRequestNode::StateRequestCallback(
    const interface_protocol::msg::MotionStateRequest::SharedPtr msg) {
  if (!msg || !IsEnabled() || !request_mailbox_) {
    return;
  }

  const auto& target = msg->target_motion_name;
  if (target != "idle" &&
      target != "passive" &&
      target != "pd_stand" &&
      target != "walk" &&
      target != "dance") {
    LOG(WARNING) << "Ignoring unknown motion state request: " << target;
    return;
  }

  request_mailbox_->Submit(target);
  LOG(INFO) << "Received motion state request: " << target;
}

}  // namespace ros2