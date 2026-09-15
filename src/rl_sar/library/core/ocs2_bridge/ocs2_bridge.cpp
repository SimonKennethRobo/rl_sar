/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#include "ocs2_bridge.hpp"

#include <cstring>
#include <cmath>
#include <iostream>
#include <optional>

#include <zmq.hpp>

#include "logger.hpp"

using namespace wbc_bridge;

namespace
{
// Poll slice for the receiver thread. Short enough that Stop() returns
// promptly, long enough that an idle link costs nothing.
constexpr int kPollTimeoutMs = 20;
}

OCS2Bridge::OCS2Bridge() = default;

OCS2Bridge::~OCS2Bridge()
{
    Stop();
}

double OCS2Bridge::Now()
{
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

bool OCS2Bridge::Start(const Config &config)
{
    if (running_.load(std::memory_order_acquire))
    {
        std::cout << LOGGER::WARNING << "[OCS2Bridge] Already running" << std::endl;
        return true;
    }

    config_ = config;

    try
    {
        context_ = std::make_unique<zmq::context_t>(1);

        // Commands: connect as SUB. The bridge binds, so rl_sar can be
        // restarted independently of the MPC.
        cmd_socket_ = std::make_unique<zmq::socket_t>(*context_, zmq::socket_type::sub);
        cmd_socket_->set(zmq::sockopt::conflate, 1);
        cmd_socket_->set(zmq::sockopt::subscribe, "");
        // CONFLATE requires the option to be set before connect, and it is
        // incompatible with a non-empty subscription filter -- hence the
        // subscribe-all above.
        cmd_socket_->connect(config_.cmd_endpoint);

        // State: bind as PUB. No queue: if the bridge is not listening the
        // samples are simply dropped.
        state_socket_ = std::make_unique<zmq::socket_t>(*context_, zmq::socket_type::pub);
        state_socket_->set(zmq::sockopt::sndhwm, 1);
        state_socket_->set(zmq::sockopt::linger, 0);
        state_socket_->bind(config_.state_endpoint);
    }
    catch (const zmq::error_t &e)
    {
        std::cout << LOGGER::ERROR << "[OCS2Bridge] Socket setup failed: " << e.what() << std::endl;
        cmd_socket_.reset();
        state_socket_.reset();
        context_.reset();
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        has_cmd_ = false;
        latest_cmd_recv_time_ = 0.0;
    }
    {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        stats_ = Stats{};
        expected_rx_seq_ = 0;
    }
    tx_seq_ = 0;

    running_.store(true, std::memory_order_release);
    rx_thread_ = std::thread(&OCS2Bridge::ReceiveLoop, this);

    std::cout << LOGGER::INFO << "[OCS2Bridge] Started"
              << " | cmd(SUB) " << config_.cmd_endpoint
              << " | state(PUB) " << config_.state_endpoint << std::endl;
    return true;
}

void OCS2Bridge::Stop()
{
    if (!running_.exchange(false, std::memory_order_acq_rel))
    {
        return;
    }

    // Closing the context unblocks the receiver's poll with ETERM.
    if (context_)
    {
        context_->shutdown();
    }
    if (rx_thread_.joinable())
    {
        rx_thread_.join();
    }

    cmd_socket_.reset();
    state_socket_.reset();
    context_.reset();

    std::cout << LOGGER::INFO << "[OCS2Bridge] Stopped" << std::endl;
}

void OCS2Bridge::PublishState(StateMsg &msg, double gait_phase_rad)
{
    if (!running_.load(std::memory_order_acquire) || !state_socket_)
    {
        return;
    }

    try
    {
        // dontwait: a blocked publisher must never stall the control loop.
        std::optional<std::size_t> sent;
        if (std::isfinite(gait_phase_rad))
        {
            PhaseStateMsg packet{};
            packet.state = msg;
            packet.gait_phase_rad = gait_phase_rad;
            FillHeader(packet.state.h, MSG_STATE_PHASE, tx_seq_++, Now());
            msg.h = packet.state.h;
            sent = state_socket_->send(zmq::const_buffer(&packet, sizeof(packet)), zmq::send_flags::dontwait);
        }
        else
        {
            FillHeader(msg.h, MSG_STATE, tx_seq_++, Now());
            sent = state_socket_->send(zmq::const_buffer(&msg, sizeof(msg)), zmq::send_flags::dontwait);
        }
        if (sent.has_value())
        {
            std::lock_guard<std::mutex> lock(stats_mutex_);
            ++stats_.tx_count;
        }
    }
    catch (const zmq::error_t &e)
    {
        if (e.num() != ETERM)
        {
            std::cout << LOGGER::WARNING << "[OCS2Bridge] State publish failed: " << e.what() << std::endl;
        }
    }
}

void OCS2Bridge::ReceiveLoop()
{
    zmq::pollitem_t items[] = {{static_cast<void *>(*cmd_socket_), 0, ZMQ_POLLIN, 0}};

    while (running_.load(std::memory_order_acquire))
    {
        try
        {
            zmq::poll(items, 1, std::chrono::milliseconds(kPollTimeoutMs));
            if (!(items[0].revents & ZMQ_POLLIN))
            {
                continue;
            }

            zmq::message_t frame;
            auto received = cmd_socket_->recv(frame, zmq::recv_flags::dontwait);
            if (!received.has_value())
            {
                continue;
            }

            CmdMsg cmd{};
            const bool size_ok = frame.size() == sizeof(CmdMsg);
            if (size_ok)
            {
                std::memcpy(&cmd, frame.data(), sizeof(CmdMsg));
            }

            if (!size_ok || !CheckHeader(cmd.h, MSG_CMD, frame.size(), sizeof(CmdMsg)))
            {
                std::lock_guard<std::mutex> lock(stats_mutex_);
                ++stats_.rx_rejected;
                // One line per bad frame would flood; the counter is the signal.
                continue;
            }

            const double now = Now();
            {
                std::lock_guard<std::mutex> lock(cmd_mutex_);
                latest_cmd_ = cmd;
                latest_cmd_recv_time_ = now;
                has_cmd_ = true;
            }
            {
                std::lock_guard<std::mutex> lock(stats_mutex_);
                if (stats_.rx_count > 0 && cmd.h.seq > expected_rx_seq_)
                {
                    stats_.rx_dropped += cmd.h.seq - expected_rx_seq_;
                }
                expected_rx_seq_ = cmd.h.seq + 1;
                ++stats_.rx_count;
                stats_.last_latency_s = now - cmd.h.stamp;
            }
        }
        catch (const zmq::error_t &e)
        {
            if (e.num() == ETERM)
            {
                break;
            }
            std::cout << LOGGER::WARNING << "[OCS2Bridge] Receive error: " << e.what() << std::endl;
        }
    }
}

bool OCS2Bridge::GetCommand(CmdMsg &out, double &age_s) const
{
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    if (!has_cmd_)
    {
        age_s = 0.0;
        return false;
    }
    out = latest_cmd_;
    age_s = Now() - latest_cmd_recv_time_;
    return true;
}

OCS2Bridge::LinkState OCS2Bridge::Link() const
{
    double age = 0.0;
    double recv_time = 0.0;
    bool has = false;
    {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        has = has_cmd_;
        recv_time = latest_cmd_recv_time_;
    }
    if (!has)
    {
        return LinkState::WAITING;
    }
    age = Now() - recv_time;
    if (age > config_.fault_timeout_s)
    {
        return LinkState::FAULT;
    }
    if (age > config_.stale_timeout_s)
    {
        return LinkState::STALE;
    }
    return LinkState::OK;
}

const char *OCS2Bridge::LinkStateName(LinkState state)
{
    switch (state)
    {
    case LinkState::WAITING: return "WAITING";
    case LinkState::OK:      return "OK";
    case LinkState::STALE:   return "STALE";
    case LinkState::FAULT:   return "FAULT";
    }
    return "?";
}

OCS2Bridge::Stats OCS2Bridge::GetStats() const
{
    std::lock_guard<std::mutex> lock(stats_mutex_);
    return stats_;
}
