/*
 * Wire protocol between the OCS2 MPC bridge and rl_sar.
 *
 * DUPLICATE COPY. The canonical file lives at
 *   go2_x5_ocs2_bridge/include/wbc_bridge/wbc_bridge_msgs.hpp
 * and is duplicated here so that rl_sar stays buildable without the ROS 2
 * workspace on the include path. Edit both, and bump kProtocolVersion on every
 * layout change -- receivers reject foreign versions rather than reinterpreting bytes.
 *
 * Transport: ZeroMQ, one PUB/SUB socket per direction, both ends CONFLATE=1 so
 * only the newest frame is ever delivered. Payload is the raw struct; sender and
 * receiver are the same host and ABI, so no serialisation library is involved.
 *
 * Clocks: every `stamp` is std::chrono::steady_clock seconds. On Linux that is
 * CLOCK_MONOTONIC, which is system-wide, so stamps from the two processes are
 * directly comparable and (recv_now - msg.stamp) is a true one-way latency.
 * Staleness must still be judged with the receiver's own clock at receive time.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef WBC_BRIDGE_MSGS_HPP
#define WBC_BRIDGE_MSGS_HPP

#include <cstdint>
#include <cstddef>

namespace wbc_bridge
{

// 'WBC' + protocol generation.
constexpr uint32_t kMagic = 0x57424301u;
constexpr uint16_t kProtocolVersion = 1;

constexpr int kNumArmDofs = 6;
constexpr int kNumLegDofs = 12;

enum MsgId : uint16_t
{
    MSG_STATE    = 1,
    MSG_CMD      = 2,
    MSG_CTRL_REQ = 3,
    MSG_CTRL_REP = 4,
    // StateMsg followed by one float64 policy gait phase [rad]. IDs 5/6 are
    // reserved by the experimental servo protocol in the bridge repository.
    MSG_STATE_PHASE = 7,
};

// What the bridge is asking rl_sar to do with the base channels. The arm
// channels are always live whenever solver_ok is set.
enum CmdMode : uint8_t
{
    // Bridge is up but not commanding: hold the current pose, zero velocity.
    // Also what rl_sar must synthesise for itself when the link goes stale.
    CMD_MODE_IDLE = 0,
    // Stage 1 bring-up: base linear velocity forced to zero by the bridge, only
    // yaw rate and the height/pitch/roll pose channels are driven.
    CMD_MODE_POSE_ONLY = 1,
    // Full whole-body: planar velocity, yaw rate, pose and arm all driven.
    CMD_MODE_FULL = 2,
};

enum CtrlCmd : uint32_t
{
    CTRL_ENABLE = 1,
    CTRL_DISABLE = 2,
    // Re-seat the MPC at the robot's measured state and restart the solver.
    CTRL_RESET = 3,
    // arg = [px, py, pz, qx, qy, qz, qw] in the OCS2 world frame.
    CTRL_SET_EE_TARGET = 4,
};

enum CtrlStatus : uint32_t
{
    CTRL_OK = 0,
    CTRL_ERR_UNKNOWN_CMD = 1,
    CTRL_ERR_NOT_READY = 2,
};

#pragma pack(push, 1)

struct MsgHeader
{
    uint32_t magic;    // kMagic
    uint16_t version;  // kProtocolVersion
    uint16_t msg_id;   // MsgId
    uint64_t seq;      // monotonically increasing per sender, for drop counting
    double   stamp;    // steady_clock seconds at send time
};

/*
 * rl_sar -> bridge. Raw measurements only: rl_sar reports the frames it
 * actually observes (Go2 trunk IMU site) and the bridge, which owns the OCS2
 * model conventions, applies the site->arm-mounting-plane offset itself.
 */
struct StateMsg
{
    MsgHeader h;

    // IMU site position in the world/odom frame [m]. In MuJoCo this is the
    // framepos sensor; on hardware it is the state estimator's output.
    float base_pos_world[3];
    // Trunk orientation, w x y z, world frame.
    float base_quat_wxyz[4];
    // Linear velocity in the TRUNK frame [m/s] -- rl_sar's obs.lin_vel.
    float base_lin_vel_body[3];
    // Angular velocity in the TRUNK frame [rad/s] -- the raw gyro.
    float base_ang_vel_body[3];

    // x5_joint1..6 [rad] / [rad/s].
    float arm_q[kNumArmDofs];
    float arm_dq[kNumArmDofs];

    // Policy order (FL, FR, RL, RR). Diagnostics only; the MPC model has no legs.
    float leg_q[kNumLegDofs];
    float leg_dq[kNumLegDofs];

    // Identifies which RLFSM state produced this sample, so the bridge can tell
    // "robot is standing and tracking" from "robot is lying down".
    uint8_t fsm_state_id;
    // rl_sar has entered the OCS2 state and wants commands.
    uint8_t bridge_enabled;
    // The locomotion policy is loaded and stepping.
    uint8_t policy_ok;
    uint8_t reserved;
};

// Backward-compatible phase extension. Legacy StateMsg remains exactly 224
// bytes; phase-aware consumers can require this message for residual models.
struct PhaseStateMsg
{
    StateMsg state;
    double gait_phase_rad;
};

/*
 * bridge -> rl_sar. Already converted into rl_sar's own conventions:
 * body-frame velocities, and a height RELATIVE to base_height_target, so the
 * consumer can drop these straight into rl.control without further maths.
 */
struct CmdMsg
{
    MsgHeader h;

    // Body-frame planar velocity [m/s] -> rl.control.x, rl.control.y.
    float base_lin_vel_body_xy[2];
    // Body-frame yaw rate [rad/s] -> rl.control.yaw.
    float base_ang_vel_body_z;

    // Offset from base_height_target [m] -> rl.control.body_height.
    float body_height_cmd;
    // [rad] -> rl.control.body_pitch / body_roll.
    float body_pitch_cmd;
    float body_roll_cmd;

    // x5_joint1..6 position targets [rad] and velocity feedforward [rad/s].
    float arm_q_cmd[kNumArmDofs];
    float arm_dq_cmd[kNumArmDofs];
    // Normalised 0 (closed) .. 1 (open). Unused until the gripper is wired up.
    float gripper_cmd;

    // MPC time this command was evaluated at, and the last solve duration [ms].
    double policy_time;
    float solve_time_ms;

    // Zero means the MPC has no valid policy yet; treat the sample as IDLE.
    uint8_t solver_ok;
    uint8_t mode;  // CmdMode
    uint8_t reserved[2];
};

struct CtrlReq
{
    MsgHeader h;
    uint32_t cmd;  // CtrlCmd
    float arg[8];
};

struct CtrlRep
{
    MsgHeader h;
    uint32_t status;  // CtrlStatus
    char text[64];
};

#pragma pack(pop)

static_assert(sizeof(MsgHeader) == 24, "MsgHeader layout changed");
static_assert(sizeof(StateMsg) == 24 + 52 + 48 + 96 + 4, "StateMsg layout changed");
static_assert(sizeof(PhaseStateMsg) == 232, "PhaseStateMsg layout changed");
static_assert(sizeof(CmdMsg) == 24 + 24 + 48 + 4 + 12 + 4, "CmdMsg layout changed");
static_assert(sizeof(CtrlReq) == 24 + 4 + 32, "CtrlReq layout changed");
static_assert(sizeof(CtrlRep) == 24 + 4 + 64, "CtrlRep layout changed");

inline void FillHeader(MsgHeader &h, MsgId id, uint64_t seq, double stamp)
{
    h.magic = kMagic;
    h.version = kProtocolVersion;
    h.msg_id = static_cast<uint16_t>(id);
    h.seq = seq;
    h.stamp = stamp;
}

// Guards against a stale peer binary and against ZMQ handing us a frame from
// some other publisher on a reused endpoint.
inline bool CheckHeader(const MsgHeader &h, MsgId expect_id, size_t payload_size,
                        size_t expect_size)
{
    return h.magic == kMagic && h.version == kProtocolVersion &&
           h.msg_id == static_cast<uint16_t>(expect_id) && payload_size == expect_size;
}

// Default endpoints. ipc:// keeps the loop off the TCP stack; switch to
// tcp://<host>:5555 etc. when the MPC runs on a separate computer.
constexpr const char *kDefaultCmdEndpoint   = "ipc:///tmp/wbc_cmd";
constexpr const char *kDefaultStateEndpoint = "ipc:///tmp/wbc_state";
constexpr const char *kDefaultCtrlEndpoint  = "ipc:///tmp/wbc_ctrl";

}  // namespace wbc_bridge

#endif  // WBC_BRIDGE_MSGS_HPP
