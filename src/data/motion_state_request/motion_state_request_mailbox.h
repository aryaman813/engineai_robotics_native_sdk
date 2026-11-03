#pragma once

#include <chrono>
#include <mutex>
#include <optional>
#include <string>
#include <utility>

namespace data {
    struct MotionStateRequest {
        std::string target_motion_name;
        std::chrono::steady_clock::time_point received_at;
    };

    class MotionStateRequestMailbox {
        public:
            void Submit(std::string target_motion_name) {
                std::lock_guard<std::mutex> lock(mutex_);

                if (pending_ &&
                    pending_ ->target_motion_name == "passive" &&
                    target_motion_name != "passive") {
                    return;
                }
                pending_ = MotionStateRequest{
                    std::move(target_motion_name),
                    std::chrono::steady_clock::now(),
                };
            }
            std::optional<MotionStateRequest> Take() {
                std::lock_guard<std::mutex> lock(mutex_);
                auto request = std::move(pending_);
                pending_.reset();
                return request;
            }

            void Clear() {
                std::lock_guard<std::mutex> lock(mutex_);
                pending_.reset();
            }
        private:
            std::mutex mutex_;
            std::optional<MotionStateRequest> pending_;
        };
}