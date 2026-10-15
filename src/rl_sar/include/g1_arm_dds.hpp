// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "rl_sdk.hpp"
#include <cmath>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>

inline void SubscribeArmDDS(RL& rl)
{
    if (!rl.params.Has("arm_dds_topic")) return;
    using Message = unitree_hg::msg::dds_::LowCmd_;
    auto indices = rl.params.Get<std::vector<int>>("arm_dds_motor_index");
    auto subscriber = std::make_shared<unitree::robot::ChannelSubscriber<Message>>(
        rl.params.Get<std::string>("arm_dds_topic"));
    subscriber->InitChannel([&rl, indices](const void* data) {
        const auto& message = *static_cast<const Message*>(data);
        std::array<float, 10> targets;
        for (int i = 0; i < 10; ++i) {
            targets[i] = message.motor_cmd()[indices[i]].q();
            if (!std::isfinite(targets[i])) return;
        }
        const float weight = message.motor_cmd()[29].q();
        if (!std::isfinite(weight)) return;
        std::lock_guard<std::mutex> lock(rl.arm_dds.mutex);
        rl.arm_dds.targets = targets;
        rl.arm_dds.weight = std::clamp(weight, 0.f, 1.f);
        rl.arm_dds.received = std::chrono::steady_clock::now();
    }, 1);
    rl.arm_dds_subscriber = subscriber;
}
