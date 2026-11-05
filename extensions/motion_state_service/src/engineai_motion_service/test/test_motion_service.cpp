#include <atomic>
#include <chrono>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <thread>
#include "engineai_motion_service/motion_service.hpp"

using namespace std::chrono_literals;
using Server = engineai_motion_service::MotionService;
using Change = Server::Change;

void Require(bool result, const std::string& message) {
  if (!result) throw std::runtime_error(message);
  std::cout << "PASS: " << message << '\n';
}

bool Eventually(const std::function<bool()>& condition) {
  const auto deadline = std::chrono::steady_clock::now() + 4s;
  while (!condition() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(20ms);
  }
  return condition();
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  const std::string url = "udpm://239.255.76.67:17771?ttl=0";
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("sdk_motion_config", TEST_CONFIG),
    rclcpp::Parameter("lcm_url", url),
    rclcpp::Parameter("request_topic", "/test/motion_request"),
    rclcpp::Parameter("service_name", "/test/change_state"),
    rclcpp::Parameter("feedback_timeout_sec", 0.15),
    rclcpp::Parameter("transition_timeout_sec", 0.7)});
  auto server = std::make_shared<Server>(options);
  auto mock = std::make_shared<rclcpp::Node>("mock_sdk");
  auto client = mock->create_client<Change>("/test/change_state");
  lcm::LCM lcm(url);
  std::mutex state_mutex;
  std::string state = "passive";
  std::atomic<bool> feedback{false}, apply{true};
  std::atomic<int> commands{0};
  auto timer = mock->create_wall_timer(20ms, [&] {
    if (!feedback) return;
    data::TaskState msg;
    std::lock_guard<std::mutex> lock(state_mutex);
    msg.current_motion_task_name = state;
    lcm.publish("task_state", &msg);
  });
  auto set_state = [&](const std::string& name) {
    std::lock_guard<std::mutex> lock(state_mutex);
    state = name;
  };
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(server);
  executor.add_node(mock);
  std::thread worker([&] { executor.spin(); });
  int result = 0;
  try {
    Require(client->wait_for_service(3s), "service discovered");
    auto send = [&](const std::string& target) {
      auto request = std::make_shared<Change::Request>();
      request->target_motion_name = target;
      return client->async_send_request(request);
    };
    auto finish = [&](auto& future) {
      Require(future.wait_for(3s) == std::future_status::ready, "bounded service response");
      return future.get();
    };
    auto call = [&](const std::string& target) {
      auto future = send(target);
      return finish(future);
    };
    Require(!call("unknown")->valid, "unknown target rejected");
    Require(!call("walk")->valid, "missing feedback rejects ordinary requests");
    auto response = call("passive");
    Require(response->valid && !response->command_sent,
            "passive is valid without feedback, but no subscriber prevents dispatch");

    auto subscription = mock->create_subscription<interface_protocol::msg::MotionStateRequest>(
        "/test/motion_request", rclcpp::QoS(1).reliable(),
        [&](interface_protocol::msg::MotionStateRequest::SharedPtr msg) {
          ++commands;
          if (apply) set_state(msg->target_motion_name);
        });
    Require(Eventually([&] { return subscription->get_publisher_count() == 1; }),
            "mock SDK subscriber discovered");
    feedback = true;
    std::this_thread::sleep_for(200ms);
    Require(!call("walk")->valid, "passive cannot skip to walk");
    Require(commands == 0, "invalid requests publish no command");
    response = call("pd_stand");
    Require(response->valid && response->command_sent && response->state_changed,
            "passive recovery to PD Stand confirmed through actual LCM feedback");
    const int before = commands;
    response = call("pd_stand");
    Require(response->valid && !response->command_sent && commands == before,
            "already-current state is a no-op");
    Require(!call("dance")->valid, "PD Stand to dance rejected by adjacency policy");
    Require(call("walk")->state_changed, "PD Stand to walk confirmed");
    Require(call("dance")->state_changed, "walk to dance confirmed");
    Require(!call("pd_stand")->valid, "dance cannot skip backward to PD Stand");
    Require(call("walk")->state_changed, "dance to walk confirmed");
    Require(call("pd_stand")->state_changed, "walk to PD Stand confirmed");
    Require(!call("idle")->valid, "adjacent step still respects SDK transition rules");

    feedback = false;
    std::this_thread::sleep_for(250ms);
    Require(!call("walk")->valid, "stale feedback rejects ordinary requests");
    response = call("passive");
    Require(response->valid && response->command_sent && !response->state_changed,
            "passive dispatches without fresh feedback; no false confirmation");
    feedback = true;
    set_state("pd_stand");
    std::this_thread::sleep_for(200ms);
    apply = false;
    response = call("walk");
    Require(response->valid && response->command_sent && !response->state_changed,
            "ignored SDK request times out instead of claiming transition success");

    const int prior = commands;
    auto pending = send("walk");
    Require(Eventually([&] { return commands > prior; }), "ordinary command dispatched");
    Require(!call("dance")->valid, "concurrent ordinary request rejected while busy");
    apply = true;
    response = call("passive");
    auto preempted = finish(pending);
    Require(response->state_changed && !preempted->state_changed &&
            preempted->message.find("preempted") != std::string::npos,
            "passive preempts pending ordinary service call");
  } catch (const std::exception& error) {
    std::cerr << "FAIL: " << error.what() << '\n';
    result = 1;
  }
  executor.cancel();
  worker.join();
  rclcpp::shutdown();
  return result;
}
