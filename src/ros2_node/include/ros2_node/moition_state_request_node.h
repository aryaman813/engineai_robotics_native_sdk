#pragma once

#include <memory>
#include <string>

#include "interface_protocol/msg/motion_state_request.hpp"
#include "ros2_node/base_node.h"

namespace data {
    class MotionStateRequestMailbox;
}

namespace ros2 {
    class MotionStateRequestNode final : public LogicNode {
        public:
            MotionStateRequestNode(
                const std::shared_ptr<data::DataStore>& data_store,
                const std::string& param_tag,
                const std::string& node_name = "motion_state_request_node");
                : LogicNode(data_store, param_tag, node_name) {}
            
            ~MotionStateRequestNode() override = default;

        private:
            bool Init() override;
            void StateRequestCallback(
                const interface_protocol::msg::MotionStateRequest::SharedPtr msg);

            rclcpp::Subscription<
                interface_protocol::msg::MotionStateRequest>::SharedPtr
                state_request_subcriber_;
            
            std::shared_ptr<data::MotionStateRequestMailbox> request_mailbox_;
    };

    REGISTER_LOGIC_NODE_TYPE(
        MotionStateRequestNode,
        "motion_state_request_node"
    );
}