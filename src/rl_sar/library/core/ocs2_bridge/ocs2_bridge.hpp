/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef OCS2_BRIDGE_HPP
#define OCS2_BRIDGE_HPP

#include <atomic>
#include <chrono>
#include <memory>
#include <limits>
#include <mutex>
#include <string>
#include <thread>

#include "wbc_bridge_msgs.hpp"

namespace zmq
{
class context_t;
class socket_t;
}

/**
 * @brief rl_sar's end of the ZeroMQ link to the OCS2 MPC bridge.
 *
 * Publishes robot state and receives whole-body commands. Both sockets are
 * CONFLATE=1: a real-time controller only ever wants the newest frame, and a
 * queue that grows under load is worse than a dropped sample.
 *
 * Threading: Start() spawns one receiver thread that owns the SUB socket.
 * PublishState() owns the PUB socket and must be called from a single thread
 * (rl_sar's loop_bridge); GetCommand() is safe from any thread.
 *
 * The link is treated as unreliable by construction. GetCommand() reports the
 * age of the newest sample and Link() classifies it, so callers never have to
 * reason about wall-clock timeouts themselves.
 */
class OCS2Bridge
{
public:
    struct Config
    {
        std::string cmd_endpoint = wbc_bridge::kDefaultCmdEndpoint;
        std::string state_endpoint = wbc_bridge::kDefaultStateEndpoint;
        // Older than this and the command is no longer trusted for base
        // velocity; the caller should coast to zero.
        double stale_timeout_s = 0.1;
        // Older than this and the link counts as lost: leave the OCS2 state.
        double fault_timeout_s = 0.5;
    };

    enum class LinkState
    {
        // No command has ever arrived; the bridge may not be running yet.
        WAITING,
        // Fresh commands, younger than stale_timeout_s.
        OK,
        // Between stale_timeout_s and fault_timeout_s: hold, do not give up.
        STALE,
        // Past fault_timeout_s: the caller must abandon OCS2 control.
        FAULT,
    };

    struct Stats
    {
        uint64_t rx_count = 0;
        // Gaps in the sender's seq. Under CONFLATE these are normal and only
        // interesting as a rate; a sudden spike means the consumer is too slow.
        uint64_t rx_dropped = 0;
        uint64_t rx_rejected = 0;  // bad magic/version/size
        uint64_t tx_count = 0;
        double last_latency_s = 0.0;
    };

    OCS2Bridge();
    ~OCS2Bridge();

    OCS2Bridge(const OCS2Bridge &) = delete;
    OCS2Bridge &operator=(const OCS2Bridge &) = delete;

    /** @return false if either socket could not be opened; the object stays unstarted. */
    bool Start(const Config &config);
    void Stop();
    bool IsRunning() const { return running_.load(std::memory_order_acquire); }

    /**
     * @brief Send one state sample. Header fields are filled in here.
     * @note Single-threaded: call only from the loop that owns the bridge.
     */
    void PublishState(wbc_bridge::StateMsg &msg,
                      double gait_phase_rad = std::numeric_limits<double>::quiet_NaN());

    /**
     * @brief Copy out the newest command received so far.
     * @param out  Filled only when the function returns true.
     * @param age_s Seconds since that command was received, on our own clock.
     * @return false if nothing has ever arrived.
     */
    bool GetCommand(wbc_bridge::CmdMsg &out, double &age_s) const;

    LinkState Link() const;
    static const char *LinkStateName(LinkState state);

    Stats GetStats() const;

    /** Seconds on the clock both ends of the link share (CLOCK_MONOTONIC). */
    static double Now();

private:
    void ReceiveLoop();

    Config config_;
    std::atomic<bool> running_{false};

    std::unique_ptr<zmq::context_t> context_;
    std::unique_ptr<zmq::socket_t> cmd_socket_;    // SUB, receiver thread only
    std::unique_ptr<zmq::socket_t> state_socket_;  // PUB, publisher thread only
    std::thread rx_thread_;

    mutable std::mutex cmd_mutex_;
    wbc_bridge::CmdMsg latest_cmd_{};
    double latest_cmd_recv_time_ = 0.0;
    bool has_cmd_ = false;

    mutable std::mutex stats_mutex_;
    Stats stats_;
    uint64_t expected_rx_seq_ = 0;

    uint64_t tx_seq_ = 0;
};

#endif // OCS2_BRIDGE_HPP
