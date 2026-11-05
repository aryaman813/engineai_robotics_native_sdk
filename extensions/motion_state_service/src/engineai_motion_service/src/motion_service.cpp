#include "engineai_motion_service/motion_service.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

namespace engineai_motion_service {

MotionService::MotionService(const rclcpp::NodeOptions& options)
    : Node("motion_state_service", options) {
  const auto config = declare_parameter<std::string>("sdk_motion_config", "");
  const auto url = declare_parameter<std::string>(
      "lcm_url", "udpm://239.255.76.67:7667?ttl=0");
  const auto topic = declare_parameter<std::string>("request_topic", "/motion/state_request");
  const auto service = declare_parameter<std::string>("service_name", "/motion/change_state");
  feedback_timeout_ = declare_parameter<double>("feedback_timeout_sec", 0.5);
  transition_timeout_ = declare_parameter<double>("transition_timeout_sec", 3.0);
  if (config.empty()) {
    throw std::runtime_error("Set sdk_motion_config to the active SDK task_motion YAML");
  }
  if (!std::isfinite(feedback_timeout_) || feedback_timeout_ <= 0 ||
      !std::isfinite(transition_timeout_) || transition_timeout_ <= 0 ||
      transition_timeout_ > 30) {
    throw std::runtime_error("Timeouts must be positive; transition timeout must be <= 30s");
  }
  const auto tasks = YAML::LoadFile(config)["tasks"];
  if (!tasks.IsSequence()) throw std::runtime_error("Motion config must contain a tasks list");
  for (const auto& task : tasks) {
    const auto name = task["motion"].as<std::string>();
    auto& edges = sdk_edges_[name];
    if (task["manual_transition"]) {
      for (const auto& target : task["manual_transition"]) edges.insert(target.as<std::string>());
    }
  }
  for (const auto& target : targets_) {
    if (!sdk_edges_.count(target)) throw std::runtime_error("Missing task in config: " + target);
  }

  lcm_ = std::make_unique<lcm::LCM>(url);
  if (!lcm_->good()) throw std::runtime_error("Cannot open LCM: " + url);
  auto* subscription = lcm_->subscribe("task_state", &MotionService::ReadState, this);
  subscription->setQueueCapacity(1);
  timer_ = create_wall_timer(std::chrono::milliseconds(10), [this] {
    // Bound work per callback so incoming traffic cannot starve ROS callbacks.
    for (int i = 0; i < 16; ++i) {
      const int result = lcm_->handleTimeout(0);
      if (result < 0) {
        RCLCPP_ERROR(get_logger(), "LCM receive error; state will expire");
        break;
      }
      if (result == 0) break;
    }
  });
  publisher_ = create_publisher<interface_protocol::msg::MotionStateRequest>(
      topic, rclcpp::QoS(1).reliable().durability_volatile());
  service_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  service_ = create_service<Change>(service,
      std::bind(&MotionService::Handle, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_group_);
  RCLCPP_INFO(get_logger(), "Service %s -> %s; feedback: %s / task_state",
              service.c_str(), topic.c_str(), url.c_str());
}

bool MotionService::Fresh() const {
  return !state_.empty() &&
      std::chrono::duration<double>(Clock::now() - state_time_).count() <= feedback_timeout_;
}

bool MotionService::Adjacent(const std::string& from, const std::string& to) const {
  // Passive recovery follows the SDK's existing idle / PD Stand entry choices.
  if (from == "passive") return to == "idle" || to == "pd_stand";
  const auto a = std::find(order_.begin(), order_.end(), from);
  const auto b = std::find(order_.begin(), order_.end(), to);
  return a != order_.end() && b != order_.end() && std::abs(a - b) == 1;
}

void MotionService::ReadState(const lcm::ReceiveBuffer*, const std::string&,
                              const data::TaskState* msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  state_ = msg->current_motion_task_name;
  state_time_ = Clock::now();
  if (busy_ && state_ == pending_target_) observed_ = true;
  changed_.notify_all();
}

void MotionService::Handle(const std::shared_ptr<Change::Request> request,
                           std::shared_ptr<Change::Response> response) {
  const auto& target = request->target_motion_name;
  std::unique_lock<std::mutex> lock(mutex_);
  response->current_motion_name = Fresh() ? state_ : "unknown";
  if (!targets_.count(target)) {
    response->message = "Invalid target; use idle, passive, pd_stand, walk, or dance";
    return;
  }
  const bool stop = target == "passive";
  if (!stop && busy_) {
    response->message = "Another transition is pending; only passive can preempt it";
    return;
  }
  if (!stop && !Fresh()) {
    response->message = "No fresh actual task_state feedback; ordinary request rejected";
    return;
  }
  if (!stop && state_ != target) {
    if (!Adjacent(state_, target)) {
      response->message = "Invalid step: require idle <-> pd_stand <-> walk <-> dance; "
                          "passive recovery permits idle or pd_stand";
      return;
    }
    const auto source = sdk_edges_.find(state_);
    if (source == sdk_edges_.end() || !source->second.count(target)) {
      response->message = "Adjacent step rejected by SDK manual_transition rules";
      return;
    }
  }
  response->valid = true;
  // Always dispatch passive, even if feedback already says passive: it can
  // cancel a conflicting command that has not yet appeared in feedback.
  if (!stop && state_ == target) {
    response->message = "Already in requested state; no command sent";
    return;
  }
  if (publisher_->get_subscription_count() == 0) {
    response->message = "Valid request, but no SDK request subscriber is discovered";
    return;
  }

  const auto generation = ++generation_;
  busy_ = true;
  observed_ = false;
  pending_target_ = target;
  interface_protocol::msg::MotionStateRequest command;
  command.target_motion_name = target;
  try {
    publisher_->publish(command);
  } catch (const std::exception& error) {
    busy_ = false;
    response->message = std::string("Publish failed: ") + error.what();
    changed_.notify_all();
    return;
  }
  response->command_sent = true;
  changed_.notify_all();  // Wake any earlier call preempted by passive.
  const auto deadline = Clock::now() + std::chrono::duration<double>(transition_timeout_);
  changed_.wait_until(lock, deadline, [&] {
    return generation != generation_ || observed_ || !rclcpp::ok();
  });
  response->current_motion_name = Fresh() ? state_ : "unknown";
  if (generation != generation_) {
    response->message = "Command sent, then preempted by a passive request";
    return;
  }
  response->state_changed = observed_;
  busy_ = false;
  response->message = observed_
      ? "Requested state observed in feedback after dispatch"
      : "Command sent, but target not confirmed before timeout; inspect FSM and input priority";
}

}  // namespace engineai_motion_service
