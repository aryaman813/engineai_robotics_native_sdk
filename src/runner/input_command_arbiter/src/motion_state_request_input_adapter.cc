#include "input_command_arbiter/motion_state_request_input_adapter.h"

#include <string>
#include <utility>
#include <vector>

#include "data/motion_state_request/motion_state_request_mailbox.h"
#include "variant_store/variant_store.h"

namespace runner {

MotionStateRequestInputAdapter::MotionStateRequestInputAdapter(
    std::string name,
    const std::shared_ptr<data::DataStore>& data_store)
    : BaseInputAdapter(std::move(name), data_store) {}

bool MotionStateRequestInputAdapter::Init() {
  mailbox_ = data::VariantStore::GetInstance()
                 .GetOrCreate<data::MotionStateRequestMailbox>(
                     "motion/state_request_mailbox");

  request_input_.Reset();
  active_ = false;
  return true;
}

InputAdapterStatus MotionStateRequestInputAdapter::Run() {
  const auto now = std::chrono::steady_clock::now();

  if (active_ && now >= expires_at_) {
    active_ = false;
    request_input_.Reset();
  }

  const auto request = mailbox_->Take();
  if (!request) {
    return InputAdapterStatus::NORMAL;
  }

  // Ignore requests that sat unconsumed for too long.
  if (now - request->received_at > std::chrono::milliseconds(500)) {
    return InputAdapterStatus::NORMAL;
  }

  // While emitting a stop, ordinary requests cannot replace it.
  if (active_ && request_input_.LB && request_input_.RB &&
      request->target_motion_name != "passive") {
    return InputAdapterStatus::NORMAL;
  }

  std::vector<std::string> keys;
  const auto& target = request->target_motion_name;

  if (target == "idle") {
    keys = {"LB", "START"};
  } else if (target == "passive") {
    keys = {"LB", "RB"};
  } else if (target == "pd_stand") {
    keys = {"LB", "A"};
  } else if (target == "walk") {
    keys = {"LB", "B"};
  } else if (target == "dance") {
    keys = {"RB", "B"};
  } else {
    return InputAdapterStatus::NORMAL;
  }

  data::GamepadTool tool;
  request_input_ = tool.ConstructGamepadInfo(keys);
  request_input_.hardware_connected = false;
  request_input_.enforce_command = false;

  // Match the existing virtual UI's approximate macro hold duration.
  expires_at_ = request->received_at + std::chrono::milliseconds(500);
  active_ = true;

  return InputAdapterStatus::NORMAL;
}

void MotionStateRequestInputAdapter::Process(data::GamepadInfo& input) {
  input = request_input_;
}

bool MotionStateRequestInputAdapter::IsActive() const {
  return active_;
}

void MotionStateRequestInputAdapter::Cancel() {
  active_ = false;
  request_input_.Reset();
  mailbox_->Clear();
}

}  // namespace runner