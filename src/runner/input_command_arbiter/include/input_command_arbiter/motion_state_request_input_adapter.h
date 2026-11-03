#pragma once

#include <chrono>
#include <memory>

#include "input_command_arbiter/base_input_adapter.h"

namespace data {
class MotionStateRequestMailbox;
}

namespace runner {

class MotionStateRequestInputAdapter final : public BaseInputAdapter {
 public:
  MotionStateRequestInputAdapter(
      std::string name,
      const std::shared_ptr<data::DataStore>& data_store);

  bool Init() override;
  InputAdapterStatus Run() override;
  void Process(data::GamepadInfo& input) override;
  bool IsActive() const override;

  void Cancel();

 private:
  std::shared_ptr<data::MotionStateRequestMailbox> mailbox_;
  data::GamepadInfo request_input_;
  bool active_{false};
  std::chrono::steady_clock::time_point expires_at_;
};

}  // namespace runner