#pragma once

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <map>
#include <mutex>
#include <set>
#include <string>
#include <vector>

#include <lcm/lcm-cpp.hpp>
#include <rclcpp/rclcpp.hpp>
#include "engineai_motion_service/srv/change_motion_state.hpp"
#include "interface_protocol/msg/motion_state_request.hpp"
#include "lcm_data/TaskState.hpp"

namespace engineai_motion_service {

class MotionService : public rclcpp::Node {
 public:
  using Change = srv::ChangeMotionState;
  explicit MotionService(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 private:
  using Clock = std::chrono::steady_clock;
  void ReadState(const lcm::ReceiveBuffer*, const std::string&, const data::TaskState*);
  void Handle(const std::shared_ptr<Change::Request>, std::shared_ptr<Change::Response>);
  bool Fresh() const;
  bool Adjacent(const std::string&, const std::string&) const;

  std::map<std::string, std::set<std::string>> sdk_edges_;
  const std::vector<std::string> order_{"idle", "pd_stand", "walk", "dance"};
  const std::set<std::string> targets_{"idle", "passive", "pd_stand", "walk", "dance"};
  std::unique_ptr<lcm::LCM> lcm_;
  rclcpp::Publisher<interface_protocol::msg::MotionStateRequest>::SharedPtr publisher_;
  rclcpp::Service<Change>::SharedPtr service_;
  rclcpp::CallbackGroup::SharedPtr service_group_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::mutex mutex_;
  std::condition_variable changed_;
  std::string state_;
  Clock::time_point state_time_{};
  double feedback_timeout_;
  double transition_timeout_;
  uint64_t generation_{0};
  bool busy_{false};
  bool observed_{false};
  std::string pending_target_;
};

}  // namespace engineai_motion_service
