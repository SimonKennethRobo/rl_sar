/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef GO2_X5_FSM_HPP
#define GO2_X5_FSM_HPP

#include "fsm.hpp"
#include "rl_sdk.hpp"

#ifdef USE_OCS2_BRIDGE
#include <cmath>
#include "ocs2_bridge.hpp"
#endif

#ifdef USE_MUJOCO
#include <algorithm>
#include <random>
#endif

// Unitree Go2 carrying a 6-DoF ARX X5 arm, driven by a RoboDuet stage-1 dog
// policy. 18 actuated joints: 12 legs (policy order FL, FR, RL, RR) followed by
// x5_joint1..6. The policy only outputs the 12 leg actions; the arm is normally
// held at its default position by a zero entry in action_scale. In MuJoCo-only
// builds it can instead be driven by a random-walk disturbance (press 'B') that
// mirrors RoboDuet's WBCEnv._apply_stage1_arm_curriculum_actions, the same
// generator stage-1 training used to teach the leg policy to reject arm motion.
namespace go2_x5_fsm
{

class RLFSMStatePassive : public RLFSMState
{
public:
    RLFSMStatePassive(RL *rl) : RLFSMState(*rl, "RLFSMStatePassive") {}

    void Enter() override
    {
        std::cout << LOGGER::NOTE << "Entered passive mode. Press '0' (Keyboard) or 'A' (Gamepad) to switch to RLFSMStateGetUp." << std::endl;
    }

    void Run() override
    {
        const int num_leg_dofs = rl.params.Get<int>("num_leg_dofs");
        for (int i = 0; i < rl.params.Get<int>("num_of_dofs"); ++i)
        {
            fsm_command->motor_command.dq[i] = 0;
            fsm_command->motor_command.tau[i] = 0;
            if (i < num_leg_dofs)
            {
                // Legs go limp so the robot can be laid down safely.
                fsm_command->motor_command.kp[i] = 0;
                fsm_command->motor_command.kd[i] = 8;
            }
            else
            {
                // Go2 passive/damping is the common safety state. The arm must
                // enter damping with the legs; use its measured position only
                // as the instantaneous command to avoid a position jump.
                fsm_command->motor_command.q[i] = fsm_state->motor_state.q[i];
                fsm_command->motor_command.kp[i] = 0;
                fsm_command->motor_command.kd[i] = 0;
            }
        }
    }

    void Exit() override {}

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A)
        {
            return "RLFSMStateGetUp";
        }
        return state_name_;
    }
};

class RLFSMStateGetUp : public RLFSMState
{
public:
    RLFSMStateGetUp(RL *rl) : RLFSMState(*rl, "RLFSMStateGetUp") {}

    float percent_pre_getup = 0.0f;
    float percent_getup = 0.0f;
    // Crouched pose (policy order: FL, FR, RL, RR legs, then x5_joint1..6).
    std::vector<float> pre_running_pos = {
        0.00, 1.36, -2.65,
        0.00, 1.36, -2.65,
        0.00, 1.36, -2.65,
        0.00, 1.36, -2.65,
        0.00, 0.00, 0.00, 0.00, 0.00, 0.00
    };
    bool stand_from_passive = true;

    void Enter() override
    {
        percent_pre_getup = 0.0f;
        percent_getup = 0.0f;
        if (rl.fsm.previous_state_->GetStateName() == "RLFSMStatePassive")
        {
            stand_from_passive = true;
        }
        else
        {
            stand_from_passive = false;
        }
        rl.now_state = *fsm_state;
        rl.start_state = rl.now_state;
    }

    void Run() override
    {
        if (stand_from_passive)
        {
            if (Interpolate(percent_pre_getup, rl.now_state.motor_state.q, pre_running_pos, 1.0f, "Pre Getting up", true)) return;
            if (Interpolate(percent_getup, pre_running_pos, rl.params.Get<std::vector<float>>("default_dof_pos"), 2.0f, "Getting up", true)) return;
        }
        else
        {
            if (Interpolate(percent_getup, rl.now_state.motor_state.q, rl.params.Get<std::vector<float>>("default_dof_pos"), 1.0f, "Getting up", true)) return;
        }
    }

    void Exit() override {}

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X)
        {
            return "RLFSMStatePassive";
        }
        if (percent_getup >= 1.0f)
        {
            if (rl.control.current_keyboard == Input::Keyboard::Num1 || rl.control.current_gamepad == Input::Gamepad::RB_DPadUp)
            {
                return "RLFSMStateRLLocomotion";
            }
            else if (rl.control.current_keyboard == Input::Keyboard::Num2 || rl.control.current_gamepad == Input::Gamepad::RB_DPadDown)
            {
                // OCS2 is an arm mode now; the leg FSM remains locomotion.
                return "RLFSMStateRLLocomotion";
            }
            else if (rl.control.current_keyboard == Input::Keyboard::Num9 || rl.control.current_gamepad == Input::Gamepad::B)
            {
                return "RLFSMStateGetDown";
            }
        }
        return state_name_;
    }
};

class RLFSMStateGetDown : public RLFSMState
{
public:
    RLFSMStateGetDown(RL *rl) : RLFSMState(*rl, "RLFSMStateGetDown") {}

    float percent_getdown = 0.0f;

    void Enter() override
    {
        percent_getdown = 0.0f;
        rl.now_state = *fsm_state;
    }

    void Run() override
    {
        Interpolate(percent_getdown, rl.now_state.motor_state.q, rl.start_state.motor_state.q, 2.0f, "Getting down", true);
    }

    void Exit() override {}

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X || percent_getdown >= 1.0f)
        {
            return "RLFSMStatePassive";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A)
        {
            return "RLFSMStateGetUp";
        }
        return state_name_;
    }
};

class RLFSMStateRLLocomotion : public RLFSMState
{
public:
    RLFSMStateRLLocomotion(RL *rl) : RLFSMState(*rl, "RLFSMStateRLLocomotion") {}

    float percent_transition = 0.0f;

#ifdef USE_MUJOCO
    // Random-walk arm disturbance, mirroring RoboDuet's
    // _apply_stage1_arm_curriculum_actions (accel -> vel -> pos, clamped to
    // joint limits with a velocity bounce on contact). MuJoCo-only: never
    // compiled into the hardware binaries.
    bool arm_perturb_enabled_ = false;
    int arm_perturb_step_ = 0;
    std::vector<float> arm_perturb_offset_;
    std::vector<float> arm_perturb_vel_;
    std::vector<float> arm_perturb_accel_;
    std::mt19937 arm_perturb_rng_{std::random_device{}()};
#endif

    void Enter() override
    {
        percent_transition = 0.0f;
        rl.episode_length_buf = 0;
        // Training resets the gait clock on episode reset; match that here so
        // the phase is deterministic every time the policy is (re-)entered.
        rl.gait_indices = 0.0f;

        // read params from yaml. Which policy directory under policy/go2_x5/
        // gets loaded is base.yaml's config_name, so switching policies is a
        // one-line yaml edit rather than a rebuild.
        rl.config_name = rl.params.Get<std::string>("config_name", "roboduet_stage1");
        std::string robot_config_path = rl.robot_name + "/" + rl.config_name;
        try
        {
            rl.InitRL(robot_config_path);
            rl.now_state = *fsm_state;
        }
        catch (const std::exception& e)
        {
            std::cout << LOGGER::ERROR << "InitRL() failed: " << e.what() << std::endl;
            rl.rl_init_done = false;
            rl.fsm.RequestStateChange("RLFSMStatePassive");
        }

#ifdef USE_MUJOCO
        // Re-entering the state (e.g. after GetDown/GetUp) should not carry a
        // stale offset into the freshly reset arm.
        arm_perturb_step_ = 0;
        const int num_arm_dofs = rl.params.Get<int>("num_arm_dofs", 0);
        arm_perturb_offset_.assign(num_arm_dofs, 0.0f);
        arm_perturb_vel_.assign(num_arm_dofs, 0.0f);
        arm_perturb_accel_.assign(num_arm_dofs, 0.0f);
        if (!arm_perturb_enabled_)
        {
            rl.ClearExternalArmTarget();
        }
#endif
    }

    void Run() override
    {
        if (!rl.rl_init_done) rl.rl_init_done = true;

#ifdef USE_MUJOCO
        StepArmPerturbation();
#endif

        std::cout << "\r\033[K" << std::flush << LOGGER::INFO << "RL Controller [" << rl.config_name << "]"
                  << " x:" << rl.control.x << " y:" << rl.control.y << " yaw:" << rl.control.yaw
                  << " pitch:" << rl.control.body_pitch << " roll:" << rl.control.body_roll
                  << " height:" << rl.control.body_height
#ifdef USE_MUJOCO
                  << " arm_perturb:" << (arm_perturb_enabled_ ? "ON" : "off")
#endif
                  << std::flush;
        RLControl();
    }

    void Exit() override
    {
        rl.rl_init_done = false;
#ifdef USE_MUJOCO
        rl.ClearExternalArmTarget();
#endif
    }

#ifdef USE_MUJOCO
    void StepArmPerturbation()
    {
        // 'N' is already the global navigation-mode toggle (rl_sdk.cpp); use 'B'.
        if (rl.control.current_keyboard == Input::Keyboard::B || rl.control.current_gamepad == Input::Gamepad::LB_Y)
        {
            arm_perturb_enabled_ = !arm_perturb_enabled_;
            if (!arm_perturb_enabled_)
            {
                rl.ClearExternalArmTarget();
            }
            std::cout << std::endl << LOGGER::NOTE << "[go2_x5] Arm perturbation "
                      << (arm_perturb_enabled_ ? "enabled" : "disabled") << std::endl;
        }
        if (!arm_perturb_enabled_)
        {
            return;
        }

        const int num_arm_dofs = rl.params.Get<int>("num_arm_dofs", 0);
        const int num_leg_dofs = rl.params.Get<int>("num_leg_dofs", 0);
        if (num_arm_dofs <= 0 || (int)arm_perturb_offset_.size() != num_arm_dofs)
        {
            return;
        }

        const float dt = rl.params.Get<float>("dt") * rl.params.Get<int>("decimation");
        // Older exports omit these MuJoCo-only settings. Match the defaults
        // used by rc_s17 and the legacy Go2 X5 config; explicit YAML values win.
        const float max_accel = rl.params.Get<float>("arm_perturb_max_accel", 3.0f);
        const float max_vel = rl.params.Get<float>("arm_perturb_max_vel", 1.0f);
        const float resample_time = rl.params.Get<float>("arm_perturb_accel_resample_time", 0.1f);
        const float zero_accel_prob = rl.params.Get<float>("arm_perturb_zero_accel_probability", 0.3f);
        const float zero_vel_prob = rl.params.Get<float>("arm_perturb_zero_vel_probability", 0.005f);
        // Physical joint ranges, in x5_joint1..6 order (go2_x5.xml).
        const std::vector<float> default_lower{-2.094f, 0.0f, 0.0f, -1.671f, -1.671f, -2.094f};
        const std::vector<float> default_upper{3.14159f, 3.665f, 3.24f, 1.671f, 1.671f, 2.094f};
        const auto lower = rl.params.Get<std::vector<float>>("arm_perturb_pos_lower", default_lower);
        const auto upper = rl.params.Get<std::vector<float>>("arm_perturb_pos_upper", default_upper);
        const auto default_dof_pos = rl.params.Get<std::vector<float>>("default_dof_pos");

        // Missing bounds use the defaults above; malformed explicit bounds
        // must still be rejected before indexing them.
        if ((int)lower.size() != num_arm_dofs || (int)upper.size() != num_arm_dofs)
        {
            std::cout << std::endl << LOGGER::WARNING << "[go2_x5] " << rl.config_name
                      << " arm_perturb_pos_lower/upper must each contain " << num_arm_dofs
                      << " entries; arm perturbation disabled" << std::endl;
            arm_perturb_enabled_ = false;
            rl.ClearExternalArmTarget();
            return;
        }

        std::uniform_real_distribution<float> unit_dist(-1.0f, 1.0f);
        std::uniform_real_distribution<float> prob_dist(0.0f, 1.0f);

        const int resample_steps = std::max(1, (int)(resample_time / dt));
        if (arm_perturb_step_ % resample_steps == 0)
        {
            for (int i = 0; i < num_arm_dofs; ++i)
            {
                arm_perturb_accel_[i] = unit_dist(arm_perturb_rng_) * max_accel;
            }
        }
        ++arm_perturb_step_;

        // One coin flip per tick (not per joint) -- matches training, where the
        // whole arm either freezes or moves together, never joint-by-joint.
        const bool zero_accel = prob_dist(arm_perturb_rng_) < zero_accel_prob;
        const bool zero_vel = prob_dist(arm_perturb_rng_) < zero_vel_prob;

        std::vector<float> target_q(num_arm_dofs), target_dq(num_arm_dofs);
        for (int i = 0; i < num_arm_dofs; ++i)
        {
            const float accel = zero_accel ? 0.0f : arm_perturb_accel_[i];
            arm_perturb_vel_[i] = std::clamp(arm_perturb_vel_[i] + accel * dt, -max_vel, max_vel);
            if (zero_vel)
            {
                arm_perturb_vel_[i] = 0.0f;
            }
            arm_perturb_offset_[i] += arm_perturb_vel_[i] * dt;

            const float arm_default = default_dof_pos[num_leg_dofs + i];
            float target = arm_default + arm_perturb_offset_[i];
            if ((target < lower[i] && arm_perturb_vel_[i] < 0.0f) || (target > upper[i] && arm_perturb_vel_[i] > 0.0f))
            {
                arm_perturb_vel_[i] *= -1.0f;
            }
            target = std::clamp(target, lower[i], upper[i]);
            arm_perturb_offset_[i] = target - arm_default;

            target_q[i] = target;
            target_dq[i] = arm_perturb_vel_[i];
        }
        rl.SetExternalArmTarget(target_q, target_dq);
    }
#endif

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X)
        {
            return "RLFSMStatePassive";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num9 || rl.control.current_gamepad == Input::Gamepad::B)
        {
            return "RLFSMStateGetDown";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A)
        {
            return "RLFSMStateGetUp";
        }
        return state_name_;
    }
};

#ifdef USE_OCS2_BRIDGE

// Whole-body manipulation: the same RoboDuet locomotion policy as
// RLFSMStateRLLocomotion, but with its base commands and the arm joint targets
// coming from an OCS2 MPC over ZeroMQ instead of the operator.
//
// The MPC solves for a 12-DOF model -- a fully actuated floating body carrying
// the ARX5 arm (go2_x5_ocs2/config/task_floating.info) -- and the bridge on the
// far side converts its solution into this robot's own conventions, so
// everything arriving here is directly assignable to rl.control.
//
// The link is assumed unreliable. Three tiers of degradation:
//   OK     -> track the MPC
//   STALE  -> coast base velocity to zero, freeze the arm where it is
//   FAULT  -> retract the arm home, then fall back to RLFSMStateRLLocomotion
class RLFSMStateOCS2Manip : public RLFSMState
{
public:
    RLFSMStateOCS2Manip(RL *rl) : RLFSMState(*rl, "RLFSMStateOCS2Manip") {}

    // Per-tick ceiling on arm joint motion, applied at the FSM rate. Sized well
    // under the ARX5's own 5.5 rad/s so a bad solve cannot snap the arm.
    static constexpr float kArmMaxJointSpeed = 1.5f;   // [rad/s]
    // How long the arm takes to fold back to default_dof_pos on the way out.
    static constexpr float kRetractSeconds = 1.5f;

    void Enter() override
    {
        rl.episode_length_buf = 0;
        rl.gait_indices = 0.0f;
        retracting_ = false;
        retract_percent_ = 0.0f;
        leave_reason_.clear();
        next_state_ = state_name_;
        last_link_ = OCS2Bridge::LinkState::WAITING;

        rl.config_name = rl.params.Get<std::string>("config_name", "roboduet_stage1");
        try
        {
            // Temporary, bundle-local workaround: a later corrected policy
            // must not inherit the old model's reversed pitch convention.
            rl.params.Set("ocs2_legacy_pitch", YAML::Node(false));
            rl.InitRL(rl.robot_name + "/" + rl.config_name);
            rl.now_state = *fsm_state;
        }
        catch (const std::exception &e)
        {
            std::cout << LOGGER::ERROR << "InitRL() failed: " << e.what() << std::endl;
            rl.rl_init_done = false;
            rl.fsm.RequestStateChange("RLFSMStatePassive");
            return;
        }

        num_arm_dofs_ = rl.params.Get<int>("num_arm_dofs", 0);
        legacy_pitch_command_ = rl.params.Get<bool>("ocs2_legacy_pitch", false);
        if (legacy_pitch_command_)
        {
            std::cout << LOGGER::NOTE
                      << "[OCS2] Temporary legacy pitch inversion enabled for "
                      << rl.config_name << std::endl;
        }
        arm_begin_ = rl.params.Get<int>("num_of_dofs") - num_arm_dofs_;
        if (num_arm_dofs_ <= 0 || arm_begin_ < 0)
        {
            std::cout << LOGGER::ERROR << "[OCS2] num_arm_dofs missing or inconsistent" << std::endl;
            rl.fsm.RequestStateChange("RLFSMStateRLLocomotion");
            return;
        }

        // Seed the arm target at where the arm actually is, so the first
        // command cannot step it.
        arm_target_.assign(num_arm_dofs_, 0.0f);
        arm_target_dq_.assign(num_arm_dofs_, 0.0f);
        arm_home_.assign(num_arm_dofs_, 0.0f);
        auto default_dof_pos = rl.params.Get<std::vector<float>>("default_dof_pos");
        for (int i = 0; i < num_arm_dofs_; ++i)
        {
            arm_target_[i] = fsm_state->motor_state.q[arm_begin_ + i];
            arm_home_[i] = default_dof_pos[arm_begin_ + i];
        }
        retract_from_ = arm_target_;
        rl.SetExternalArmTarget(arm_target_, arm_target_dq_);

        ZeroBaseCommand();

        OCS2Bridge::Config config;
        if (!bridge_.Start(config))
        {
            std::cout << LOGGER::ERROR << "[OCS2] Bridge failed to start" << std::endl;
            rl.fsm.RequestStateChange("RLFSMStateRLLocomotion");
            return;
        }

        std::cout << LOGGER::NOTE << "Entered OCS2 whole-body manipulation. "
                  << "Press '1' to hand control back to the operator." << std::endl;
    }

    void Run() override
    {
        if (!rl.rl_init_done) rl.rl_init_done = true;

        PublishState();

        wbc_bridge::CmdMsg cmd{};
        double age = 0.0;
        const bool have_cmd = bridge_.GetCommand(cmd, age);
        const OCS2Bridge::LinkState link = bridge_.Link();

        if (link != last_link_)
        {
            std::cout << std::endl << LOGGER::INFO << "[OCS2] Link "
                      << OCS2Bridge::LinkStateName(last_link_) << " -> "
                      << OCS2Bridge::LinkStateName(link) << std::endl;
            last_link_ = link;
        }

        if (retracting_)
        {
            RunRetract();
        }
        else if (link == OCS2Bridge::LinkState::OK && have_cmd && cmd.solver_ok)
        {
            ApplyCommand(cmd);
        }
        else if (link == OCS2Bridge::LinkState::FAULT)
        {
            BeginRetract("link lost");
        }
        else
        {
            // WAITING, STALE, or a solver that has not converged yet: the safe
            // action is identical -- stop translating, hold the arm still.
            CoastBaseCommand();
            std::fill(arm_target_dq_.begin(), arm_target_dq_.end(), 0.0f);
        }

        rl.SetExternalArmTarget(arm_target_, arm_target_dq_);
        PrintStatus(link, age);
        RLControl();
    }

    void Exit() override
    {
        bridge_.Stop();
        rl.ClearExternalArmTarget();
        rl.rl_init_done = false;
        ZeroBaseCommand();
    }

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X)
        {
            // Emergency stop wins over a graceful retract.
            return "RLFSMStatePassive";
        }
        if (!retracting_)
        {
            if (rl.control.current_keyboard == Input::Keyboard::Num1 || rl.control.current_gamepad == Input::Gamepad::RB_DPadUp)
            {
                BeginRetract("operator");
                next_state_ = "RLFSMStateRLLocomotion";
            }
            else if (rl.control.current_keyboard == Input::Keyboard::Num9 || rl.control.current_gamepad == Input::Gamepad::B)
            {
                BeginRetract("operator");
                next_state_ = "RLFSMStateGetDown";
            }
        }
        if (retracting_ && retract_percent_ >= 1.0f)
        {
            return next_state_;
        }
        return state_name_;
    }

private:
    void ZeroBaseCommand()
    {
        rl.control.x = 0.0f;
        rl.control.y = 0.0f;
        rl.control.yaw = 0.0f;
        rl.control.body_pitch = 0.0f;
        rl.control.body_roll = 0.0f;
        rl.control.body_height = 0.0f;
    }

    // Bleed the velocity commands out over ~0.2 s rather than stepping them to
    // zero, which the policy reads as a hard stop.
    void CoastBaseCommand()
    {
        const float decay = 0.97f;
        rl.control.x *= decay;
        rl.control.y *= decay;
        rl.control.yaw *= decay;
    }

    void ClampControl(float &value, const std::string &key) const
    {
        if (rl.params.Has(key))
        {
            auto limit = rl.params.Get<std::vector<float>>(key);
            value = std::min(std::max(value, limit[0]), limit[1]);
        }
    }

    void ApplyCommand(const wbc_bridge::CmdMsg &cmd)
    {
        // CMD_MODE_POSE_ONLY is the stage-1 bring-up mode: the bridge has
        // already zeroed the planar velocity, but zero it here too so a
        // mislabelled message cannot make the robot walk.
        const bool drive_planar = (cmd.mode == wbc_bridge::CMD_MODE_FULL);
        rl.control.x = drive_planar ? cmd.base_lin_vel_body_xy[0] : 0.0f;
        rl.control.y = drive_planar ? cmd.base_lin_vel_body_xy[1] : 0.0f;
        rl.control.yaw = cmd.base_ang_vel_body_z;
        rl.control.body_height = cmd.body_height_cmd;
        // MPC uses physical RPY; the selected legacy policy expects the
        // opposite pitch command. Keep its trained observations untouched.
        rl.control.body_pitch = legacy_pitch_command_ ? -cmd.body_pitch_cmd : cmd.body_pitch_cmd;
        rl.control.body_roll = cmd.body_roll_cmd;

        // KeyboardInterface() clamps at 20 Hz; this loop writes at the control
        // rate, so it has to clamp for itself.
        ClampControl(rl.control.x, "limit_vel_x");
        ClampControl(rl.control.y, "limit_vel_y");
        ClampControl(rl.control.yaw, "limit_vel_yaw");
        ClampControl(rl.control.body_pitch, "limit_body_pitch");
        ClampControl(rl.control.body_roll, "limit_body_roll");
        ClampControl(rl.control.body_height, "limit_body_height");

        const float max_step = kArmMaxJointSpeed * rl.params.Get<float>("dt");
        for (int i = 0; i < num_arm_dofs_; ++i)
        {
            const float delta = cmd.arm_q_cmd[i] - arm_target_[i];
            arm_target_[i] += std::min(std::max(delta, -max_step), max_step);
            arm_target_dq_[i] = rl.params.Get<bool>("native_mrt", false)
                ? std::clamp(cmd.arm_dq_cmd[i], -kArmMaxJointSpeed, kArmMaxJointSpeed)
                : cmd.arm_dq_cmd[i];
        }
    }

    void BeginRetract(const std::string &reason)
    {
        if (retracting_) return;
        retracting_ = true;
        retract_percent_ = 0.0f;
        retract_from_ = arm_target_;
        leave_reason_ = reason;
        if (next_state_ == state_name_)
        {
            next_state_ = "RLFSMStateRLLocomotion";
        }
        std::cout << std::endl << LOGGER::WARNING << "[OCS2] Retracting arm (" << reason
                  << "), then -> " << next_state_ << std::endl;
    }

    void RunRetract()
    {
        CoastBaseCommand();
        rl.control.body_height = 0.0f;
        rl.control.body_pitch = 0.0f;
        rl.control.body_roll = 0.0f;

        retract_percent_ += rl.params.Get<float>("dt") / kRetractSeconds;
        if (retract_percent_ > 1.0f) retract_percent_ = 1.0f;
        for (int i = 0; i < num_arm_dofs_; ++i)
        {
            arm_target_[i] = (1.0f - retract_percent_) * retract_from_[i] + retract_percent_ * arm_home_[i];
            arm_target_dq_[i] = 0.0f;
        }
    }

    void PublishState()
    {
        wbc_bridge::StateMsg msg{};
        for (int i = 0; i < 3; ++i)
        {
            msg.base_pos_world[i] = fsm_state->base.position[i];
            msg.base_lin_vel_body[i] = fsm_state->base.lin_vel[i];
            msg.base_ang_vel_body[i] = fsm_state->imu.gyroscope[i];
        }
        for (int i = 0; i < 4; ++i)
        {
            msg.base_quat_wxyz[i] = fsm_state->imu.quaternion[i];
        }
        for (int i = 0; i < num_arm_dofs_ && i < wbc_bridge::kNumArmDofs; ++i)
        {
            msg.arm_q[i] = fsm_state->motor_state.q[arm_begin_ + i];
            msg.arm_dq[i] = fsm_state->motor_state.dq[arm_begin_ + i];
        }
        for (int i = 0; i < arm_begin_ && i < wbc_bridge::kNumLegDofs; ++i)
        {
            msg.leg_q[i] = fsm_state->motor_state.q[i];
            msg.leg_dq[i] = fsm_state->motor_state.dq[i];
        }
        msg.fsm_state_id = 1;
        msg.bridge_enabled = retracting_ ? 0 : 1;
        msg.policy_ok = rl.rl_init_done ? 1 : 0;
        bridge_.PublishState(msg, static_cast<double>(rl.gait_indices) * 6.283185307179586);
    }

    void PrintStatus(OCS2Bridge::LinkState link, double age)
    {
        if (++print_divider_ < 40) return;  // ~5 Hz at the 200 Hz control rate
        print_divider_ = 0;
        const auto stats = bridge_.GetStats();
        std::cout << "\r\033[K" << std::flush << LOGGER::INFO
                  << "OCS2 [" << OCS2Bridge::LinkStateName(link) << "]"
                  << (retracting_ ? " RETRACT" : "")
                  << " age:" << std::fixed << std::setprecision(1) << age * 1e3 << "ms"
                  << " lat:" << stats.last_latency_s * 1e3 << "ms"
                  << " rx:" << stats.rx_count << " drop:" << stats.rx_dropped
                  << " rej:" << stats.rx_rejected
                  << std::setprecision(2)
                  << " | x:" << rl.control.x << " y:" << rl.control.y << " yaw:" << rl.control.yaw
                  << " h:" << rl.control.body_height
                  << " arm0:" << arm_target_[0] << std::flush;
    }

    OCS2Bridge bridge_;
    OCS2Bridge::LinkState last_link_ = OCS2Bridge::LinkState::WAITING;

    int num_arm_dofs_ = 0;
    bool legacy_pitch_command_ = false;
    int arm_begin_ = 0;
    std::vector<float> arm_target_;
    std::vector<float> arm_target_dq_;
    std::vector<float> arm_home_;
    std::vector<float> retract_from_;

    bool retracting_ = false;
    float retract_percent_ = 0.0f;
    std::string leave_reason_;
    std::string next_state_ = "RLFSMStateOCS2Manip";

    int print_divider_ = 0;
};

#endif // USE_OCS2_BRIDGE

} // namespace go2_x5_fsm

class Go2X5FSMFactory : public FSMFactory
{
public:
    Go2X5FSMFactory(const std::string& initial) : initial_state_(initial) {}
    std::shared_ptr<FSMState> CreateState(void *context, const std::string &state_name) override
    {
        RL *rl = static_cast<RL *>(context);
        if (state_name == "RLFSMStatePassive")
            return std::make_shared<go2_x5_fsm::RLFSMStatePassive>(rl);
        else if (state_name == "RLFSMStateGetUp")
            return std::make_shared<go2_x5_fsm::RLFSMStateGetUp>(rl);
        else if (state_name == "RLFSMStateGetDown")
            return std::make_shared<go2_x5_fsm::RLFSMStateGetDown>(rl);
        else if (state_name == "RLFSMStateRLLocomotion")
            return std::make_shared<go2_x5_fsm::RLFSMStateRLLocomotion>(rl);
#ifdef USE_OCS2_BRIDGE
        else if (state_name == "RLFSMStateOCS2Manip")
            return std::make_shared<go2_x5_fsm::RLFSMStateOCS2Manip>(rl);
#endif
        return nullptr;
    }
    std::string GetType() const override { return "go2_x5"; }
    std::vector<std::string> GetSupportedStates() const override
    {
        return {
            "RLFSMStatePassive",
            "RLFSMStateGetUp",
            "RLFSMStateGetDown",
            "RLFSMStateRLLocomotion",
#ifdef USE_OCS2_BRIDGE
            "RLFSMStateOCS2Manip",
#endif
        };
    }
    std::string GetInitialState() const override { return initial_state_; }
private:
    std::string initial_state_;
};

REGISTER_FSM_FACTORY(Go2X5FSMFactory, "RLFSMStatePassive")

#endif // GO2_X5_FSM_HPP
