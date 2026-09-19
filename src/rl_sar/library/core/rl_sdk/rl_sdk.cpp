/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rl_sdk.hpp"
#include <cmath>

#include <array>
#include <map>

void RL::StateController(const RobotState<float>* state, RobotCommand<float>* command)
{
    auto updateState = [&](std::shared_ptr<FSMState> statePtr)
    {
        if (auto rl_fsm_state = std::dynamic_pointer_cast<RLFSMState>(statePtr))
        {
            rl_fsm_state->fsm_state = state;
            rl_fsm_state->fsm_command = command;
        }
    };
    for (auto& pair : fsm.states_)
    {
        updateState(pair.second);
    }

    fsm.Run();

    this->motiontime++;

    if (this->control.current_keyboard == Input::Keyboard::W)
    {
        this->control.x += 0.1f;
    }
    if (this->control.current_keyboard == Input::Keyboard::S)
    {
        this->control.x -= 0.1f;
    }
    if (this->control.current_keyboard == Input::Keyboard::A)
    {
        this->control.y += 0.1f;
    }
    if (this->control.current_keyboard == Input::Keyboard::D)
    {
        this->control.y -= 0.1f;
    }
    if (this->control.current_keyboard == Input::Keyboard::Q)
    {
        this->control.yaw += 0.1f;
    }
    if (this->control.current_keyboard == Input::Keyboard::E)
    {
        this->control.yaw -= 0.1f;
    }
    // Body pose commands. Only observed by policies that ask for them
    // (RoboDuet's roboduet/dog_commands); harmless for every other robot.
    if (this->control.current_keyboard == Input::Keyboard::T)
    {
        this->control.body_pitch += 0.05f;
    }
    if (this->control.current_keyboard == Input::Keyboard::G)
    {
        this->control.body_pitch -= 0.05f;
    }
    if (this->control.current_keyboard == Input::Keyboard::Y)
    {
        this->control.body_roll += 0.05f;
    }
    if (this->control.current_keyboard == Input::Keyboard::H)
    {
        this->control.body_roll -= 0.05f;
    }
    if (this->control.current_keyboard == Input::Keyboard::U)
    {
        this->control.body_height += 0.02f;
    }
    if (this->control.current_keyboard == Input::Keyboard::J)
    {
        this->control.body_height -= 0.02f;
    }
    if (this->control.current_keyboard == Input::Keyboard::Space)
    {
        this->control.x = 0.0f;
        this->control.y = 0.0f;
        this->control.yaw = 0.0f;
        this->control.body_pitch = 0.0f;
        this->control.body_roll = 0.0f;
        this->control.body_height = 0.0f;
    }

    // Clamp every command to the range the policy was trained against.
    // Absent keys leave the command unclamped, so this is a no-op for robots
    // whose config does not declare limits.
    auto clamp_command = [&](float &value, const std::string &key)
    {
        if (this->params.Has(key))
        {
            auto limit = this->params.Get<std::vector<float>>(key);
            value = clamp(value, limit[0], limit[1]);
        }
    };
    clamp_command(this->control.x, "limit_vel_x");
    clamp_command(this->control.y, "limit_vel_y");
    clamp_command(this->control.yaw, "limit_vel_yaw");
    clamp_command(this->control.body_pitch, "limit_body_pitch");
    clamp_command(this->control.body_roll, "limit_body_roll");
    clamp_command(this->control.body_height, "limit_body_height");
    if (this->control.current_keyboard == Input::Keyboard::N || this->control.current_gamepad == Input::Gamepad::X)
    {
        this->control.navigation_mode = !this->control.navigation_mode;
        std::cout << std::endl << LOGGER::INFO << "Navigation mode: " << (this->control.navigation_mode ? "ON" : "OFF") << std::endl;
    }
}

std::vector<float> RL::ComputeObservation()
{
    std::vector<std::vector<float>> obs_list;
    const auto observation_terms = this->params.Get<std::vector<std::string>>("observations");
    // One command snapshot and one stand decision for both command and clock
    // observations. Keep the configured walking frequency intact for restart.
    const std::array<float, 6> dog_command = {
        control.x, control.y, control.yaw, control.body_pitch, control.body_roll, control.body_height};
    const float command_norm = std::sqrt(dog_command[0] * dog_command[0] +
                                        dog_command[1] * dog_command[1] +
                                        dog_command[2] * dog_command[2]);
    const bool standing = command_norm < 0.1f;
    float effective_gait_frequency = 0.0f;
    if (std::find(observation_terms.begin(), observation_terms.end(), "roboduet/dog_commands") != observation_terms.end() ||
        std::find(observation_terms.begin(), observation_terms.end(), "roboduet/clock_inputs") != observation_terms.end())
    {
        // Older bundles encode dynamic gait through the five extra commands.
        const auto dynamic_gait_setting = params.Get<YAML::Node>("use_dynamic_gait");
        const bool dynamic_gait = dynamic_gait_setting && !dynamic_gait_setting.IsNull()
            ? params.Get<bool>("use_dynamic_gait")
            : params.Get<std::vector<float>>("dog_commands_scale").size() > 6;
        effective_gait_frequency = params.Get<bool>("servo_observation_timing", false)
            ? params.Get<float>("gait_frequency")
            : (dynamic_gait && standing ? 0.0f : params.Get<float>("gait_frequency"));
    }
    const std::array<float, 5> response_command = {
        control.x, control.y, control.yaw, control.body_height, control.body_pitch};
    if (response_enabled_ && !response_initializing_ && episode_length_buf != response_last_step_
        && (!params.Get<bool>("servo_observation_timing", false) || servo_advancing_))
    {
        const auto euler = QuaternionToEuler(obs.base_quat);
        const float height = obs.body_pose_actual.size() == 3 ? obs.body_pose_actual[0] : obs.base_height.at(0);
        const float pitch = obs.body_pose_actual.size() == 3 ? obs.body_pose_actual[1] : euler[1];
        response_observation_.Step(response_command, {
            obs.lin_vel.at(0), obs.lin_vel.at(1), obs.ang_vel.at(2),
            height - params.Get<float>("base_height_target"), pitch});
        response_last_step_ = episode_length_buf;
    }

    for (const std::string &observation : observation_terms)
    {
        // ============= Base Observations =============
        if (observation == "lin_vel")
        {
            obs_list.push_back(this->obs.lin_vel * this->params.Get<float>("lin_vel_scale"));
        }
        else if (observation == "ang_vel")
        {
            // In ROS1 Gazebo, the coordinate system for angular velocity is in the world coordinate system.
            // In ROS2 Gazebo, mujoco and real robot, the coordinate system for angular velocity is in the body coordinate system.
            if (this->ang_vel_axis == "body")
            {
                obs_list.push_back(this->obs.ang_vel * this->params.Get<float>("ang_vel_scale"));
            }
            else if (this->ang_vel_axis == "world")
            {
                obs_list.push_back(QuatRotateInverse(this->obs.base_quat, this->obs.ang_vel) * this->params.Get<float>("ang_vel_scale"));
            }
        }
        else if (observation == "gravity_vec")
        {
            obs_list.push_back(QuatRotateInverse(this->obs.base_quat, this->obs.gravity_vec));
        }
        else if (observation == "commands")
        {
            obs_list.push_back(this->obs.commands * this->params.Get<std::vector<float>>("commands_scale"));
        }
        else if (observation == "robot_lab/velocity_pose_commands" ||
                 observation == "robot_lab/velocity_pose_commands_6d")
        {
            // IsaacLab UniformVelocityPoseCommand order:
            // [vx, vy, yaw_rate, absolute_height, roll, pitch, relative_yaw].
            // rl_sar exposes height as an operator offset around the training
            // nominal, while this policy was trained with relative_yaw == 0.
            std::vector<float> command{
                this->control.x,
                this->control.y,
                this->control.yaw,
                this->params.Get<float>("base_height_target") + this->control.body_height,
                this->control.body_roll,
                this->control.body_pitch};
            // v4_63 drops the unused pose-yaw channel. Legacy bundles retain it.
            if (observation == "robot_lab/velocity_pose_commands") command.push_back(0.0f);
            obs_list.push_back(command);
        }
        else if (observation == "dof_pos")
        {
            std::vector<float> dof_pos_rel = this->obs.dof_pos - this->params.Get<std::vector<float>>("default_dof_pos");
            for (int i : this->params.Get<std::vector<int>>("wheel_indices"))
            {
                dof_pos_rel[i] = 0.0f;
            }
            obs_list.push_back(dof_pos_rel * this->params.Get<float>("dof_pos_scale"));
        }
        else if (observation == "dof_vel")
        {
            obs_list.push_back(this->obs.dof_vel * this->params.Get<float>("dof_vel_scale"));
        }
        else if (observation == "actions")
        {
            obs_list.push_back(this->obs.actions);
        }
        // ============= RoboDuet Observations =============
        // RoboDuet's dog policy interleaves leg and arm blocks with command /
        // gait-clock / tracking blocks, so the generic dof_pos|dof_vel|actions
        // terms (which always span all num_of_dofs) cannot express it. These
        // terms slice the leg and arm ranges out explicitly. Layout mirrors
        // WBCEnv.get_dog_observations() term for term.
        else if (observation == "roboduet/leg_dof_pos")
        {
            const int num_leg_dofs = this->params.Get<int>("num_leg_dofs");
            auto default_dof_pos = this->params.Get<std::vector<float>>("default_dof_pos");
            std::vector<float> leg_dof_pos_rel(num_leg_dofs);
            for (int i = 0; i < num_leg_dofs; ++i)
            {
                leg_dof_pos_rel[i] = this->obs.dof_pos[i] - default_dof_pos[i];
            }
            obs_list.push_back(leg_dof_pos_rel * this->params.Get<float>("dof_pos_scale"));
        }
        else if (observation == "roboduet/leg_dof_vel")
        {
            const int num_leg_dofs = this->params.Get<int>("num_leg_dofs");
            std::vector<float> leg_dof_vel(this->obs.dof_vel.begin(), this->obs.dof_vel.begin() + num_leg_dofs);
            obs_list.push_back(leg_dof_vel * this->params.Get<float>("dof_vel_scale"));
        }
        else if (observation == "roboduet/leg_actions")
        {
            // obs.actions is zero-padded up to num_of_dofs; the policy only ever
            // produced the leading num_leg_dofs entries, so slice them back out.
            const int num_leg_dofs = this->params.Get<int>("num_leg_dofs");
            obs_list.push_back(std::vector<float>(this->obs.actions.begin(), this->obs.actions.begin() + num_leg_dofs));
        }
        else if (observation == "roboduet/arm_dof_pos")
        {
            const int num_leg_dofs = this->params.Get<int>("num_leg_dofs");
            const int num_arm_dofs = this->params.Get<int>("num_arm_dofs");
            auto default_dof_pos = this->params.Get<std::vector<float>>("default_dof_pos");
            std::vector<float> arm_dof_pos_rel(num_arm_dofs);
            for (int i = 0; i < num_arm_dofs; ++i)
            {
                arm_dof_pos_rel[i] = this->obs.dof_pos[num_leg_dofs + i] - default_dof_pos[num_leg_dofs + i];
            }
            obs_list.push_back(arm_dof_pos_rel * this->params.Get<float>("dof_pos_scale"));
        }
        else if (observation == "roboduet/arm_dof_vel")
        {
            const int num_leg_dofs = this->params.Get<int>("num_leg_dofs");
            const int num_arm_dofs = this->params.Get<int>("num_arm_dofs");
            std::vector<float> arm_dof_vel(this->obs.dof_vel.begin() + num_leg_dofs,
                                           this->obs.dof_vel.begin() + num_leg_dofs + num_arm_dofs);
            obs_list.push_back(arm_dof_vel * this->params.Get<float>("dof_vel_scale"));
        }
        else if (observation == "robot_lab/arm_dof_vel")
        {
            const int num_leg_dofs = this->params.Get<int>("num_leg_dofs");
            const int num_arm_dofs = this->params.Get<int>("num_arm_dofs");
            std::vector<float> arm_dof_vel(this->obs.dof_vel.begin() + num_leg_dofs,
                                           this->obs.dof_vel.begin() + num_leg_dofs + num_arm_dofs);
            obs_list.push_back(arm_dof_vel * this->params.Get<float>("arm_dof_vel_scale"));
        }
        else if (observation == "roboduet/dog_commands")
        {
            // [x_vel, y_vel, yaw_vel, body_pitch, body_roll, body_height]
            // followed, when the policy was trained with dynamic gait, by the
            // 5 gait commands [gait_frequency, footswing_height, stance_width,
            // stance_length, gait_duration], held at the constants the export
            // script recorded (InitRL unpacks those out of dog_commands_extra).
            // dog_commands_scale is exactly as wide as the command vector the
            // policy trained on, so a run without dynamic gait stops after the
            // operator's six. gait_frequency/gait_duration are read from the
            // same params the clock uses below, so this observation can never
            // drift out of sync with the actual gait clock.
            std::vector<float> dog_commands(dog_command.begin(), dog_command.end());
            const auto& dog_commands_scale = this->params.Get<std::vector<float>>("dog_commands_scale");
            if (this->params.Get<bool>("omit_height", false) && dog_commands.size() >= 6)
            {
                dog_commands.erase(dog_commands.begin() + 5);
            }
            if (dog_commands_scale.size() > dog_commands.size())
            {
                std::vector<float> gait_commands = {
                    effective_gait_frequency,
                    this->params.Get<float>("footswing_height"),
                    this->params.Get<float>("stance_width"),
                    this->params.Get<float>("stance_length"),
                    this->params.Get<float>("gait_duration")};
                gait_commands.resize(dog_commands_scale.size() - dog_commands.size(), 0.0f);
                dog_commands.insert(dog_commands.end(), gait_commands.begin(), gait_commands.end());
            }
            obs_list.push_back(dog_commands * dog_commands_scale);
        }
        else if (observation == "roboduet/arm_commands")
        {
            // Stage 1 keeps global_switch closed, so the arm command slot is
            // zero-filled in training. The slot must still occupy its width.
            obs_list.push_back(std::vector<float>(this->params.Get<int>("arm_num_commands"), 0.0f));
        }
        else if (observation == "roboduet/clock_inputs")
        {
            if (params.Get<bool>("servo_observation_timing", false) && !servo_advancing_)
            {
                obs_list.push_back(servo_clock_);
                continue;
            }
            // Dynamic-gait training sets the frequency command to zero for
            // standing samples. Fixed-gait policies retain their fixed clock.
            const float gait_duration = this->params.Get<float>("gait_duration");
            const float policy_dt = this->params.Get<float>("dt") * this->params.Get<int>("decimation");
            // Older/frozen RoboDuet exports store this as
            // [phases, offsets, bounds], while newer exports use named keys.
            // Accept both so changing policy bundles does not fail in InitRL.
            const auto gait_phases = this->params.Get<YAML::Node>("gait_phases");
            float phases, offsets, bounds;
            if (gait_phases.IsSequence())
            {
                const auto values = this->params.Get<std::vector<float>>("gait_phases");
                if (values.size() != 3)
                {
                    throw std::runtime_error("gait_phases must contain [phases, offsets, bounds]");
                }
                phases = values[0];
                offsets = values[1];
                bounds = values[2];
            }
            else
            {
                const auto values = this->params.Get<std::map<std::string, float>>("gait_phases");
                phases = values.at("phases");
                offsets = values.at("offsets");
                bounds = values.at("bounds");
            }

            this->gait_indices = std::fmod(this->gait_indices + policy_dt * effective_gait_frequency, 1.0f);

            std::vector<float> foot_indices = {
                this->gait_indices + phases + offsets + bounds,
                this->gait_indices + offsets,
                this->gait_indices + bounds,
                this->gait_indices + phases};

            // Stand is the trained 0.25 foot phase, not an all-zero clock.
            std::vector<float> clock_inputs(4, 0.0f);
            for (int i = 0; i < 4; ++i)
            {
                float idx = standing ? 0.25f : std::fmod(foot_indices[i], 1.0f);
                if (idx < 0.0f) idx += 1.0f;
                idx = (idx < gait_duration)
                          ? idx * (0.5f / gait_duration)
                          : 0.5f + (idx - gait_duration) * (0.5f / (1.0f - gait_duration));
                clock_inputs[i] = std::sin(2.0f * 3.14159265f * idx);
            }
            servo_clock_ = clock_inputs;
            obs_list.push_back(clock_inputs);
        }
        else if (observation == "roboduet/base_lin_vel")
        {
            // Zero-filled slot when the policy was trained with
            // dog.observe_lin_vel=False; the width never changes.
            if (this->params.Get<bool>("observe_lin_vel", true))
            {
                obs_list.push_back(this->obs.lin_vel * this->params.Get<float>("lin_vel_scale"));
            }
            else
            {
                obs_list.push_back(std::vector<float>(3, 0.0f));
            }
        }
        else if (observation == "roboduet/body_pose_actual")
        {
            // [base height, pitch, roll] -- note this order differs from the
            // command order [pitch, roll, height].
            if (this->params.Get<bool>("observe_pose_actual", true))
            {
                std::vector<float> pose_actual;
                if (this->obs.body_pose_actual.size() == 3)
                {
                    pose_actual = this->obs.body_pose_actual;
                }
                else
                {
                    std::vector<float> euler = QuaternionToEuler(this->obs.base_quat); // [roll, pitch, yaw]
                    pose_actual = {this->obs.base_height[0], euler[1], euler[0]};
                }
                if (this->params.Get<bool>("omit_height", false))
                {
                    obs_list.push_back(std::vector<float>{
                        pose_actual[1] * this->params.Get<float>("body_pitch_cmd_scale"),
                        pose_actual[2] * this->params.Get<float>("body_roll_cmd_scale")});
                }
                else
                {
                    obs_list.push_back(std::vector<float>{
                        pose_actual[0] * this->params.Get<float>("body_height_cmd_scale"),
                        pose_actual[1] * this->params.Get<float>("body_pitch_cmd_scale"),
                        pose_actual[2] * this->params.Get<float>("body_roll_cmd_scale")});
                }
            }
            else
            {
                obs_list.push_back(std::vector<float>(3, 0.0f));
            }
        }
        else if (observation == "roboduet/body_pose_error")
        {
            if (this->params.Get<bool>("observe_track_error", true))
            {
                std::vector<float> pose_actual;
                if (this->obs.body_pose_actual.size() == 3)
                {
                    pose_actual = this->obs.body_pose_actual;
                }
                else
                {
                    std::vector<float> euler = QuaternionToEuler(this->obs.base_quat);
                    pose_actual = {this->obs.base_height[0], euler[1], euler[0]};
                }
                const float height_target = this->params.Get<float>("base_height_target") + this->control.body_height;
                if (this->params.Get<bool>("omit_height", false))
                {
                    obs_list.push_back(std::vector<float>{
                        (this->control.body_pitch - pose_actual[1]) * this->params.Get<float>("body_pitch_cmd_scale"),
                        (this->control.body_roll - pose_actual[2]) * this->params.Get<float>("body_roll_cmd_scale")});
                }
                else
                {
                    obs_list.push_back(std::vector<float>{
                        (height_target - pose_actual[0]) * this->params.Get<float>("body_height_cmd_scale"),
                        (this->control.body_pitch - pose_actual[1]) * this->params.Get<float>("body_pitch_cmd_scale"),
                        (this->control.body_roll - pose_actual[2]) * this->params.Get<float>("body_roll_cmd_scale")});
                }
            }
            else
            {
                obs_list.push_back(std::vector<float>(3, 0.0f));
            }
        }
        else if (observation == "roboduet/velocity_error")
        {
            if (this->params.Get<bool>("observe_track_error", true))
            {
                const float lin_vel_scale = this->params.Get<float>("lin_vel_scale");
                const float ang_vel_scale = this->params.Get<float>("ang_vel_scale");
                obs_list.push_back(std::vector<float>{
                    (this->control.x - this->obs.lin_vel[0]) * lin_vel_scale,
                    (this->control.y - this->obs.lin_vel[1]) * lin_vel_scale,
                    (this->control.yaw - this->obs.ang_vel[2]) * ang_vel_scale});
            }
            else
            {
                obs_list.push_back(std::vector<float>(3, 0.0f));
            }
        }
        else if (observation == "roboduet/reference_state")
        {
            obs_list.push_back(response_observation_.State());
        }
        else if (observation == "roboduet/reference_rate")
        {
            obs_list.push_back(response_observation_.Rate());
        }
        else if (observation == "roboduet/reference_minus_cmd")
        {
            obs_list.push_back(response_observation_.Error(response_command));
        }
        else if (observation == "roboduet/ee_pos_in_base")
        {
            obs_list.push_back(response_observation_.EndEffector(obs.dof_pos));
        }
        else if (observation == "roboduet/response_deviation")
        {
            obs_list.push_back(response_observation_.Deviation());
        }
        // ============= Other Observations =============
        else if (observation == "whole_body_tracking/motion_command")
        {
            std::vector<float> motion_cmd;
            if (this->motion_loader)
            {
                auto joint_pos_sdk = this->motion_loader->GetJointPos();
                auto joint_vel_sdk = this->motion_loader->GetJointVel();
                auto joint_mapping = this->params.Get<std::vector<int>>("joint_mapping");
                std::vector<float> joint_pos_training(joint_mapping.size());
                std::vector<float> joint_vel_training(joint_mapping.size());
                for (size_t i = 0; i < joint_mapping.size(); ++i)
                {
                    joint_pos_training[i] = joint_pos_sdk[joint_mapping[i]];
                    joint_vel_training[i] = joint_vel_sdk[joint_mapping[i]];
                }
                motion_cmd.insert(motion_cmd.end(), joint_pos_training.begin(), joint_pos_training.end());
                motion_cmd.insert(motion_cmd.end(), joint_vel_training.begin(), joint_vel_training.end());
            }
            else
            {
                motion_cmd.resize(this->params.Get<int>("num_of_dofs") * 2, 0.0f);
            }
            obs_list.push_back(motion_cmd);
        }
        else if (observation == "whole_body_tracking/motion_anchor_ori_b")
        {
            std::vector<float> anchor_ori(6, 0.0f);
            if (this->motion_loader)
            {
                auto waist_sdk_indices = this->params.Get<std::vector<int>>("waist_joint_indices");
                std::vector<float> waist_angles = {
                    this->obs.dof_pos[InverseJointMapping(waist_sdk_indices[0])],
                    this->obs.dof_pos[InverseJointMapping(waist_sdk_indices[1])],
                    this->obs.dof_pos[InverseJointMapping(waist_sdk_indices[2])]
                };
                std::vector<float> robot_torso_quat_w = MotionLoader::ComputeTorsoQuat(this->obs.base_quat, waist_angles);
                std::vector<float> ref_torso_quat_w = this->motion_loader->GetAnchorQuat();
                std::vector<float> init_quat = this->motion_loader->GetInitQuat();
                std::vector<float> motion_anchor_quat_w = QuaternionMultiply(init_quat, ref_torso_quat_w);
                std::vector<float> robot_quat_inv = QuaternionConjugate(robot_torso_quat_w);
                std::vector<float> relative_quat = QuaternionMultiply(robot_quat_inv, motion_anchor_quat_w);
                std::vector<float> rot_matrix = QuaternionToRotationMatrix(relative_quat);
                anchor_ori = MatrixFirstTwoColumns(rot_matrix);
            }
            obs_list.push_back(anchor_ori);
        }
        else if (observation == "RoboMimic_Deploy/phase")
        {
            float motion_time = this->episode_length_buf * this->params.Get<float>("dt") * this->params.Get<int>("decimation");
            float count = motion_time;
            float phase = count / this->motion_length;
            std::vector<float> phase_vec = {phase};
            obs_list.push_back(phase_vec);
        }
        else
        {
            throw std::runtime_error("Unsupported observation term '" + observation + "' in policy config");
        }
    }

    this->obs_dims.clear();
    for (const auto& obs : obs_list)
    {
       this->obs_dims.push_back(obs.size());
    }

    std::vector<float> obs;
    for (const auto& obs_vec : obs_list)
    {
        obs.insert(obs.end(), obs_vec.begin(), obs_vec.end());
    }
    if (this->params.Has("num_observations"))
    {
        const int expected = this->params.Get<int>("num_observations");
        if (static_cast<int>(obs.size()) != expected)
        {
            throw std::runtime_error(
                "Observation width mismatch: built " + std::to_string(obs.size()) +
                ", config expects " + std::to_string(expected));
        }
    }
    std::vector<float> clamped_obs = clamp(obs, -this->params.Get<float>("clip_obs"), this->params.Get<float>("clip_obs"));
    return clamped_obs;
}

void RL::InitObservations()
{
    this->obs.lin_vel = {0.0f, 0.0f, 0.0f};
    this->obs.ang_vel = {0.0f, 0.0f, 0.0f};
    this->obs.gravity_vec = {0.0f, 0.0f, -1.0f};
    this->obs.commands = {0.0f, 0.0f, 0.0f};
    this->obs.base_quat = {0.0f, 0.0f, 0.0f, 1.0f};
    this->obs.base_height = {this->params.Get<float>("base_height_target")};
    this->obs.body_pose_actual.clear();
    this->obs.dof_pos = this->params.Get<std::vector<float>>("default_dof_pos");
    this->obs.dof_vel.clear();
    this->obs.dof_vel.resize(this->params.Get<int>("num_of_dofs"), 0.0f);
    this->obs.actions.clear();
    this->obs.actions.resize(this->params.Get<int>("num_of_dofs"), 0.0f);
    this->ComputeObservation();
}

void RL::InitOutputs()
{
    int num_of_dofs = this->params.Get<int>("num_of_dofs");
    this->output_dof_tau.clear();
    this->output_dof_tau.resize(num_of_dofs, 0.0f);
    this->output_dof_pos = this->params.Get<std::vector<float>>("default_dof_pos");
    this->output_dof_vel.clear();
    this->output_dof_vel.resize(num_of_dofs, 0.0f);
}

void RL::InitControl()
{
    this->control.x = 0.0f;
    this->control.y = 0.0f;
    this->control.yaw = 0.0f;
    this->control.body_pitch = 0.0f;
    this->control.body_roll = 0.0f;
    this->control.body_height = 0.0f;
}

void RL::InitJointNum(size_t num_joints)
{
    this->robot_state.motor_state.resize(num_joints);
    this->start_state.motor_state.resize(num_joints);
    this->now_state.motor_state.resize(num_joints);
    this->robot_command.motor_command.resize(num_joints);
}

void RL::InitRL(std::string robot_config_path)
{
    std::lock_guard<std::mutex> lock(this->model_mutex);

    // A bundle missing these settings must not inherit them from the previous
    // policy after a runtime switch.
    this->params.Set("response_observation", YAML::Node());
    // Missing metadata must fall back to this bundle's layout after a switch.
    this->params.Set("use_dynamic_gait", YAML::Node());
    this->params.Set("servo_observation_timing", YAML::Node(false));
    this->params.Set("native_mrt", YAML::Node(false));
    this->params.Set("policy_base_at_trunk", YAML::Node(false));
    this->ReadYaml(robot_config_path, "config.yaml");

    const auto terms = params.Get<std::vector<std::string>>("observations");
    const std::vector<std::string> response_terms = {
        "roboduet/reference_state", "roboduet/reference_rate", "roboduet/reference_minus_cmd",
        "roboduet/ee_pos_in_base", "roboduet/response_deviation"};
    response_enabled_ = std::any_of(terms.begin(), terms.end(), [&](const std::string& term) {
        return std::find(response_terms.begin(), response_terms.end(), term) != response_terms.end();
    });
    if (response_enabled_)
    {
        response_observation_.Configure(params.Get<YAML::Node>("response_observation"),
            params.Get<float>("dt") * params.Get<int>("decimation"), params.Get<int>("num_of_dofs"));
    }
    response_last_step_ = ~0ULL;
    servo_last_step_ = ~0ULL;
    servo_clock_.assign(4, 0.0f);
    gait_indices = 0.0f;

    // Newer exports pack the frozen gait command slots into a single list,
    // [gait_frequency, footswing_height, stance_width, stance_length,
    // gait_duration]; unpack it into the named params so the observation, the
    // gait clock and the joylink gait tuning all read one format regardless of
    // which exporter wrote the config.
    const auto& dog_commands_extra = this->params.Get<std::vector<float>>("dog_commands_extra");
    if (!dog_commands_extra.empty())
    {
        static const std::vector<std::string> gait_command_keys = {"gait_frequency", "footswing_height", "stance_width", "stance_length", "gait_duration"};
        if (dog_commands_extra.size() != gait_command_keys.size())
        {
            throw std::runtime_error("dog_commands_extra must hold " + std::to_string(gait_command_keys.size()) + " values, got " + std::to_string(dog_commands_extra.size()));
        }
        for (size_t i = 0; i < gait_command_keys.size(); ++i)
        {
            this->params.Set(gait_command_keys[i], YAML::Node(dog_commands_extra[i]));
        }
    }

    // init joint num first
    this->InitJointNum(this->params.Get<int>("num_of_dofs"));

    // init rl
    this->InitControl();
    response_initializing_ = true;
    this->InitObservations();
    response_initializing_ = false;
    this->InitOutputs();

    // init obs history
    const auto& observations_history = this->params.Get<std::vector<int>>("observations_history");  // avoid dangling reference
    if (!observations_history.empty())
    {
        int history_length = *std::max_element(observations_history.begin(), observations_history.end()) + 1;
        this->history_obs_buf = ObservationBuffer(1, this->obs_dims, history_length, this->params.Get<std::string>("observations_history_priority"));
    }

    // init model
    std::string model_path = std::string(POLICY_DIR) + "/" + robot_config_path + "/" + this->params.Get<std::string>("model_name");
    this->model = InferenceRuntime::ModelFactory::load_model(model_path);
    if (!this->model)
    {
        throw std::runtime_error("Failed to load model from: " + model_path);
    }
}

void RL::InitRLFromBundle(const std::string& config_path, const std::string& model_path)
{
    std::lock_guard<std::mutex> lock(this->model_mutex);
    this->params.Set("response_observation", YAML::Node());
    this->params.Set("use_dynamic_gait", YAML::Node());
    this->params.Set("servo_observation_timing", YAML::Node(false));
    this->params.Set("native_mrt", YAML::Node(false));
    this->params.Set("policy_base_at_trunk", YAML::Node(false));

    const YAML::Node file = YAML::LoadFile(config_path);
    YAML::Node config = file;
    // Exported configs are keyed by ``go2_x5/<bundle>``. Also accept a plain
    // mapping so callers can provide a minimal private bundle.
    if (file.IsMap() && file.size() == 1 && file.begin()->second.IsMap())
        config = file.begin()->second;
    if (!config.IsMap())
        throw std::runtime_error("policy config must contain a YAML mapping: " + config_path);
    for (auto it = config.begin(); it != config.end(); ++it)
        this->params.Set(it->first.as<std::string>(), it->second);
    // Exported policy bundles may omit simulator timing because the training
    // environment owns it. Headless MuJoCo evaluation needs an explicit,
    // stable default matching rl_sar's Go2 deployment.
    if (!this->params.Has("dt")) this->params.Set("dt", YAML::Node(0.005f));
    if (!this->params.Has("decimation")) this->params.Set("decimation", YAML::Node(4));

    const auto terms = params.Get<std::vector<std::string>>("observations");
    const std::vector<std::string> response_terms = {
        "roboduet/reference_state", "roboduet/reference_rate", "roboduet/reference_minus_cmd",
        "roboduet/ee_pos_in_base", "roboduet/response_deviation"};
    response_enabled_ = std::any_of(terms.begin(), terms.end(), [&](const std::string& term) {
        return std::find(response_terms.begin(), response_terms.end(), term) != response_terms.end();
    });
    if (response_enabled_)
        response_observation_.Configure(params.Get<YAML::Node>("response_observation"),
            params.Get<float>("dt") * params.Get<int>("decimation"), params.Get<int>("num_of_dofs"));
    response_last_step_ = ~0ULL;
    servo_last_step_ = ~0ULL;
    servo_clock_.assign(4, 0.0f);
    gait_indices = 0.0f;

    const auto dog_commands_extra = this->params.Get<std::vector<float>>("dog_commands_extra");
    if (!dog_commands_extra.empty()) {
        static const std::vector<std::string> keys = {"gait_frequency", "footswing_height", "stance_width", "stance_length", "gait_duration"};
        if (dog_commands_extra.size() != keys.size())
            throw std::runtime_error("dog_commands_extra must contain five values");
        for (size_t i = 0; i < keys.size(); ++i)
            this->params.Set(keys[i], YAML::Node(dog_commands_extra[i]));
    }

    this->InitJointNum(this->params.Get<int>("num_of_dofs"));
    this->InitControl();
    response_initializing_ = true;
    this->InitObservations();
    response_initializing_ = false;
    this->InitOutputs();
    const auto observations_history = this->params.Get<std::vector<int>>("observations_history");
    if (!observations_history.empty()) {
        const int history_length = *std::max_element(observations_history.begin(), observations_history.end()) + 1;
        this->history_obs_buf = ObservationBuffer(1, this->obs_dims, history_length,
                                                  this->params.Get<std::string>("observations_history_priority"));
    }
    this->model = InferenceRuntime::ModelFactory::load_model(model_path);
    if (!this->model)
        throw std::runtime_error("Failed to load model from: " + model_path);
}

void RL::ComputeOutput(const std::vector<float> &actions, std::vector<float> &output_dof_pos, std::vector<float> &output_dof_vel, std::vector<float> &output_dof_tau)
{
    std::vector<float> actions_scaled = actions * this->params.Get<std::vector<float>>("action_scale");
    std::vector<float> pos_actions_scaled = actions_scaled;
    std::vector<float> vel_actions_scaled(actions.size(), 0.0f);
    for (int i : this->params.Get<std::vector<int>>("wheel_indices"))
    {
        pos_actions_scaled[i] = 0.0f;
        vel_actions_scaled[i] = actions_scaled[i];
    }
    std::vector<float> all_actions_scaled = pos_actions_scaled + vel_actions_scaled;
    output_dof_pos = pos_actions_scaled + this->params.Get<std::vector<float>>("default_dof_pos");
    output_dof_vel = vel_actions_scaled;
    std::vector<float> tau_target = all_actions_scaled + this->params.Get<std::vector<float>>("default_dof_pos");

    // Must happen before the torque is formed, so the position loop below tracks
    // the planner's target rather than default_dof_pos.
    this->ApplyExternalArmTarget(output_dof_pos, output_dof_vel, tau_target);

    output_dof_tau = this->params.Get<std::vector<float>>("rl_kp") * (tau_target - this->obs.dof_pos) - this->params.Get<std::vector<float>>("rl_kd") * this->obs.dof_vel;
    output_dof_tau = clamp(output_dof_tau, -this->params.Get<std::vector<float>>("torque_limits"), this->params.Get<std::vector<float>>("torque_limits"));
}

void RL::SetExternalArmTarget(const std::vector<float> &q, const std::vector<float> &dq)
{
    const int num_arm_dofs = this->params.Get<int>("num_arm_dofs", 0);
    if (num_arm_dofs <= 0 || (int)q.size() != num_arm_dofs || (int)dq.size() != num_arm_dofs)
    {
        std::cout << LOGGER::ERROR << "SetExternalArmTarget(): expected " << num_arm_dofs
                  << " joints, got " << q.size() << "/" << dq.size() << std::endl;
        return;
    }
    {
        std::lock_guard<std::mutex> lock(this->external_arm_mutex_);
        this->external_arm_q_ = q;
        this->external_arm_dq_ = dq;
    }
    this->use_external_arm_.store(true, std::memory_order_release);
}

void RL::ClearExternalArmTarget()
{
    this->use_external_arm_.store(false, std::memory_order_release);
}

bool RL::HasExternalArmTarget() const
{
    return this->use_external_arm_.load(std::memory_order_acquire);
}

void RL::ApplyExternalBaseCommand(const std::array<float, 6> &cmd)
{
    auto limited = [this](float value, const std::string &key) -> float
    {
        if (!std::isfinite(value)) return 0.0f;
        if (!this->params.Has(key)) return value;
        const auto limit = this->params.Get<std::vector<float>>(key);
        return std::clamp(value, limit[0], limit[1]);
    };
    this->control.x = limited(cmd[0], "limit_vel_x");
    this->control.y = limited(cmd[1], "limit_vel_y");
    this->control.yaw = limited(cmd[2], "limit_vel_yaw");
    this->control.body_height = limited(cmd[3], "limit_body_height");
    this->control.body_pitch = limited(cmd[4], "limit_body_pitch");
    this->control.body_roll = limited(cmd[5], "limit_body_roll");
}

void RL::CoastExternalBaseCommand()
{
    const float decay = 0.97f;
    this->control.x *= decay;
    this->control.y *= decay;
    this->control.yaw *= decay;
}

void RL::ReleaseExternalBaseCommand()
{
    this->control.x = 0.0f;
    this->control.y = 0.0f;
    this->control.yaw = 0.0f;
    this->control.body_height = 0.0f;
    this->control.body_pitch = 0.0f;
    this->control.body_roll = 0.0f;
}

void RL::ApplyExternalArmTarget(std::vector<float> &output_dof_pos, std::vector<float> &output_dof_vel, std::vector<float> &tau_target)
{
    if (!this->use_external_arm_.load(std::memory_order_acquire))
    {
        return;
    }

    const int num_of_dofs = this->params.Get<int>("num_of_dofs");
    const int num_arm_dofs = this->params.Get<int>("num_arm_dofs", 0);
    // Policy order puts the legs first, so the arm is the trailing block.
    const int arm_begin = num_of_dofs - num_arm_dofs;
    if (num_arm_dofs <= 0 || arm_begin < 0 || (int)output_dof_pos.size() < num_of_dofs)
    {
        return;
    }

    std::lock_guard<std::mutex> lock(this->external_arm_mutex_);
    if ((int)this->external_arm_q_.size() != num_arm_dofs)
    {
        return;
    }
    for (int i = 0; i < num_arm_dofs; ++i)
    {
        output_dof_pos[arm_begin + i] = this->external_arm_q_[i];
        output_dof_vel[arm_begin + i] = this->external_arm_dq_[i];
        tau_target[arm_begin + i] = this->external_arm_q_[i];
    }
}

int RL::InverseJointMapping(int idx) const
{
    auto joint_mapping = this->params.Get<std::vector<int>>("joint_mapping");
    for (size_t i = 0; i < joint_mapping.size(); ++i) {
        if (joint_mapping[i] == idx) return (int)i;
    }
    return -1;
}

void RL::TorqueProtect(const std::vector<float>& origin_output_dof_tau)
{
    std::vector<int> out_of_range_indices;
    std::vector<float> out_of_range_values;
    for (size_t i = 0; i < origin_output_dof_tau.size(); ++i)
    {
        float torque_value = origin_output_dof_tau[i];
        float limit_lower = -this->params.Get<std::vector<float>>("torque_limits")[i];
        float limit_upper = this->params.Get<std::vector<float>>("torque_limits")[i];

        if (torque_value < limit_lower || torque_value > limit_upper)
        {
            out_of_range_indices.push_back(i);
            out_of_range_values.push_back(torque_value);
        }
    }
    if (!out_of_range_indices.empty())
    {
        for (size_t i = 0; i < out_of_range_indices.size(); ++i)
        {
            int index = out_of_range_indices[i];
            float value = out_of_range_values[i];
            float limit_lower = -this->params.Get<std::vector<float>>("torque_limits")[index];
            float limit_upper = this->params.Get<std::vector<float>>("torque_limits")[index];

            std::cout << LOGGER::WARNING << "Torque(" << index + 1 << ")=" << value << " out of range(" << limit_lower << ", " << limit_upper << ")" << std::endl;
        }
        // Just a reminder, no protection
        // this->control.SetKeyboard(Input::Keyboard::P);
        std::cout << LOGGER::INFO << "Switching to STATE_POS_GETDOWN"<< std::endl;
    }
}

void RL::AttitudeProtect(const std::vector<float> &quaternion, float pitch_threshold, float roll_threshold)
{
    // Use QuaternionToEuler from vector_math.hpp
    std::vector<float> euler = QuaternionToEuler(quaternion);
    float roll = euler[0] * 57.2958f;   // Convert to degrees
    float pitch = euler[1] * 57.2958f;

    if (std::fabs(roll) > roll_threshold)
    {
        this->control.SetKeyboard(Input::Keyboard::P);
        std::cout << LOGGER::WARNING << "Roll exceeds " << roll_threshold << " degrees. Current: " << roll << " degrees." << std::endl;
    }
    if (std::fabs(pitch) > pitch_threshold)
    {
        this->control.SetKeyboard(Input::Keyboard::P);
        std::cout << LOGGER::WARNING << "Pitch exceeds " << pitch_threshold << " degrees. Current: " << pitch << " degrees." << std::endl;
    }
}

#include <termios.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>

static int kbhit()
{
    termios original_term{};
    if (tcgetattr(STDIN_FILENO, &original_term) != 0) return -1;

    termios new_term = original_term;
    new_term.c_lflag &= ~(ICANON | ECHO);  // Disable canonical mode and echo
    new_term.c_cc[VMIN] = 0;   // Non-blocking read
    new_term.c_cc[VTIME] = 0;  // No timeout
    if (tcsetattr(STDIN_FILENO, TCSANOW, &new_term) != 0) return -1;

    // Non-blocking read of a single character
    char c;
    int result = read(STDIN_FILENO, &c, 1);
    tcsetattr(STDIN_FILENO, TCSANOW, &original_term);

    return (result == 1) ? (unsigned char)c : -1;
}

// Maps one typed character to a key event. Shared by the terminal reader and
// the /go2_x5/fsm/key topic so both drive the FSM through the same path.
bool RL::PressKeyChar(int c)
{
    switch (c)
    {
    case '0': this->control.SetKeyboard(Input::Keyboard::Num0); break;
    case '1': this->control.SetKeyboard(Input::Keyboard::Num1); break;
    case '2': this->control.SetKeyboard(Input::Keyboard::Num2); break;
    case '3': this->control.SetKeyboard(Input::Keyboard::Num3); break;
    case '4': this->control.SetKeyboard(Input::Keyboard::Num4); break;
    case '5': this->control.SetKeyboard(Input::Keyboard::Num5); break;
    case '6': this->control.SetKeyboard(Input::Keyboard::Num6); break;
    case '7': this->control.SetKeyboard(Input::Keyboard::Num7); break;
    case '8': this->control.SetKeyboard(Input::Keyboard::Num8); break;
    case '9': this->control.SetKeyboard(Input::Keyboard::Num9); break;
    case 'a': case 'A': this->control.SetKeyboard(Input::Keyboard::A); break;
    case 'b': case 'B': this->control.SetKeyboard(Input::Keyboard::B); break;
    case 'c': case 'C': this->control.SetKeyboard(Input::Keyboard::C); break;
    case 'd': case 'D': this->control.SetKeyboard(Input::Keyboard::D); break;
    case 'e': case 'E': this->control.SetKeyboard(Input::Keyboard::E); break;
    case 'f': case 'F': this->control.SetKeyboard(Input::Keyboard::F); break;
    case 'g': case 'G': this->control.SetKeyboard(Input::Keyboard::G); break;
    case 'h': case 'H': this->control.SetKeyboard(Input::Keyboard::H); break;
    case 'i': case 'I': this->control.SetKeyboard(Input::Keyboard::I); break;
    case 'j': case 'J': this->control.SetKeyboard(Input::Keyboard::J); break;
    case 'k': case 'K': this->control.SetKeyboard(Input::Keyboard::K); break;
    case 'l': case 'L': this->control.SetKeyboard(Input::Keyboard::L); break;
    case 'm': case 'M': this->control.SetKeyboard(Input::Keyboard::M); break;
    case 'n': case 'N': this->control.SetKeyboard(Input::Keyboard::N); break;
    case 'o': case 'O': this->control.SetKeyboard(Input::Keyboard::O); break;
    case 'p': case 'P': this->control.SetKeyboard(Input::Keyboard::P); break;
    case 'q': case 'Q': this->control.SetKeyboard(Input::Keyboard::Q); break;
    case 'r': case 'R': this->control.SetKeyboard(Input::Keyboard::R); break;
    case 's': case 'S': this->control.SetKeyboard(Input::Keyboard::S); break;
    case 't': case 'T': this->control.SetKeyboard(Input::Keyboard::T); break;
    case 'u': case 'U': this->control.SetKeyboard(Input::Keyboard::U); break;
    case 'v': case 'V': this->control.SetKeyboard(Input::Keyboard::V); break;
    case 'w': case 'W': this->control.SetKeyboard(Input::Keyboard::W); break;
    case 'x': case 'X': this->control.SetKeyboard(Input::Keyboard::X); break;
    case 'y': case 'Y': this->control.SetKeyboard(Input::Keyboard::Y); break;
    case 'z': case 'Z': this->control.SetKeyboard(Input::Keyboard::Z); break;
    case ' ': this->control.SetKeyboard(Input::Keyboard::Space); break;
    case '\n': case '\r': this->control.SetKeyboard(Input::Keyboard::Enter); break;
    default: return false;
    }
    return true;
}

void RL::InjectKey(const std::string &key)
{
    if (key == "UP") this->control.SetKeyboard(Input::Keyboard::Up);
    else if (key == "DOWN") this->control.SetKeyboard(Input::Keyboard::Down);
    else if (key == "LEFT") this->control.SetKeyboard(Input::Keyboard::Left);
    else if (key == "RIGHT") this->control.SetKeyboard(Input::Keyboard::Right);
    else if (key == "ESC") this->control.SetKeyboard(Input::Keyboard::Escape);
    else if (key == "ENTER") this->control.SetKeyboard(Input::Keyboard::Enter);
    else if (key == "SPACE") this->control.SetKeyboard(Input::Keyboard::Space);
    else if (key.size() == 1) this->PressKeyChar(static_cast<unsigned char>(key[0]));
}

void RL::KeyboardInterface()
{
    int c = kbhit();
    if (c <= 0) return;
    if (c != 27)
    {
        this->PressKeyChar(c);
        return;
    }
    // Escape sequence (arrow keys on Unix/Linux/macOS)
    char seq[2];
    if (read(STDIN_FILENO, &seq[0], 1) == 1 && seq[0] == '[')
    {
        if (read(STDIN_FILENO, &seq[1], 1) == 1)
        {
            switch (seq[1])
            {
            case 'A': this->control.SetKeyboard(Input::Keyboard::Up); break;
            case 'B': this->control.SetKeyboard(Input::Keyboard::Down); break;
            case 'C': this->control.SetKeyboard(Input::Keyboard::Right); break;
            case 'D': this->control.SetKeyboard(Input::Keyboard::Left); break;
            default: break;
            }
        }
    }
    else
    {
        // Plain escape key
        this->control.SetKeyboard(Input::Keyboard::Escape);
    }
}

template <typename T>
std::vector<T> ReadVectorFromYaml(const YAML::Node &node)
{
    std::vector<T> values;
    for (const auto &val : node)
    {
        values.push_back(val.as<T>());
    }
    return values;
}

void RL::ReadYaml(const std::string& file_path, const std::string& file_name)
{
    std::string config_path = std::string(POLICY_DIR) + "/" + file_path + "/" + file_name;
    YAML::Node config;
    try
    {
        config = YAML::LoadFile(config_path)[file_path];
    }
    catch (YAML::BadFile &e)
    {
        std::cout << LOGGER::ERROR << "The file '" << config_path << "' does not exist" << std::endl;
        return;
    }

    for (auto it = config.begin(); it != config.end(); ++it)
    {
        std::string key = it->first.as<std::string>();
        this->params.Set(key, it->second);
    }
}

void RL::CSVInit(std::string robot_path)
{
    csv_filename = std::string(POLICY_DIR) + "/" + robot_path + "/motor";

    // Uncomment these lines if need timestamp for file name
    // auto now = std::chrono::system_clock::now();
    // std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    // std::stringstream ss;
    // ss << std::put_time(std::localtime(&now_c), "%Y%m%d%H%M%S");
    // std::string timestamp = ss.str();
    // csv_filename += "_" + timestamp;

    csv_filename += ".csv";
    std::ofstream file(csv_filename.c_str());

    for(int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i) { file << "tau_cal_" << i << ","; }
    for(int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i) { file << "tau_est_" << i << ","; }
    for(int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i) { file << "joint_pos_" << i << ","; }
    for(int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i) { file << "joint_pos_target_" << i << ","; }
    for(int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i) { file << "joint_vel_" << i << ","; }

    file << std::endl;

    file.close();
}

void RL::CSVLogger(const std::vector<float>& torque, const std::vector<float>& tau_est, const std::vector<float>& joint_pos, const std::vector<float>& joint_pos_target, const std::vector<float>& joint_vel)
{
    std::ofstream file(csv_filename.c_str(), std::ios_base::app);

    for(int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i) { file << torque[i] << ","; }
    for(int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i) { file << tau_est[i] << ","; }
    for(int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i) { file << joint_pos[i] << ","; }
    for(int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i) { file << joint_pos_target[i] << ","; }
    for(int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i) { file << joint_vel[i] << ","; }

    file << std::endl;

    file.close();
}

// Advance the frozen RC_s17 gait/response state from the command that was
// applied during the completed policy interval. Ordinary policies do not call
// this path.
void RL::AdvanceServoObservation(const std::vector<float>& applied_command)
{
    if (!params.Get<bool>("servo_observation_timing", false) || applied_command.size() != 11)
        throw std::runtime_error("AdvanceServoObservation requires servo mode and 11 dog commands");
    if (servo_last_step_ == episode_length_buf) return;
    const auto saved_control = control;
    const std::vector<std::string> keys = {
        "gait_frequency", "footswing_height", "stance_width", "stance_length", "gait_duration"};
    std::vector<float> saved;
    for (const auto& key : keys) saved.push_back(params.Get<float>(key));
    control.x = applied_command[0];
    control.y = applied_command[1];
    control.yaw = applied_command[2];
    control.body_pitch = applied_command[3];
    control.body_roll = applied_command[4];
    control.body_height = applied_command[5];
    for (size_t i=0; i<keys.size(); ++i) params.Set(keys[i], YAML::Node(applied_command[6+i]));
    servo_advancing_ = true;
    try
    {
        ComputeObservation();
    }
    catch (...)
    {
        servo_advancing_ = false;
        control = saved_control;
        for (size_t i=0; i<keys.size(); ++i) params.Set(keys[i], YAML::Node(saved[i]));
        throw;
    }
    servo_advancing_ = false;
    control = saved_control;
    for (size_t i=0; i<keys.size(); ++i) params.Set(keys[i], YAML::Node(saved[i]));
    servo_last_step_ = episode_length_buf;
}

bool RLFSMState::Interpolate(
    float& percent,
    const std::vector<float>& start_pos,
    const std::vector<float>& target_pos,
    float duration_seconds,
    const std::string& description,
    bool use_fixed_gains)
{
    if (percent >= 1.0f)
    {
        return false;
    }

    if (percent == 0.0f)
    {
        float max_diff = 0.0f;
        for (size_t i = 0; i < start_pos.size() && i < target_pos.size(); ++i)
        {
            max_diff = std::max(max_diff, std::abs(start_pos[i] - target_pos[i]));
        }

        if (max_diff < 0.1f)
        {
            percent = 1.0f;
        }
    }

    int required_frames = std::max(1, static_cast<int>(std::ceil(duration_seconds / rl.params.Get<float>("dt"))));
    float step = 1.0f / required_frames;

    percent += step;
    percent = std::min(percent, 1.0f);

    auto kp = use_fixed_gains ? rl.params.Get<std::vector<float>>("fixed_kp") : rl.params.Get<std::vector<float>>("rl_kp");
    auto kd = use_fixed_gains ? rl.params.Get<std::vector<float>>("fixed_kd") : rl.params.Get<std::vector<float>>("rl_kd");

    for (int i = 0; i < rl.params.Get<int>("num_of_dofs"); ++i)
    {
        fsm_command->motor_command.q[i] = (1 - percent) * start_pos[i] + percent * target_pos[i];
        fsm_command->motor_command.dq[i] = 0;
        fsm_command->motor_command.kp[i] = kp[i];
        fsm_command->motor_command.kd[i] = kd[i];
        fsm_command->motor_command.tau[i] = 0;
    }

    if (!description.empty())
    {
        LOGGER::PrintProgress(percent, description);
    }

    if (percent >= 1.0f)
    {
        return false;
    }

    return true;
}

void RLFSMState::RLControl()
{
    std::vector<float> _output_dof_pos, _output_dof_vel;
    if (rl.output_dof_pos_queue.try_pop(_output_dof_pos) && rl.output_dof_vel_queue.try_pop(_output_dof_vel))
    {
        for (int i = 0; i < rl.params.Get<int>("num_of_dofs"); ++i)
        {
            if (!_output_dof_pos.empty())
            {
                fsm_command->motor_command.q[i] = _output_dof_pos[i];
            }
            if (!_output_dof_vel.empty())
            {
                fsm_command->motor_command.dq[i] = _output_dof_vel[i];
            }
            fsm_command->motor_command.kp[i] = rl.params.Get<std::vector<float>>("rl_kp")[i];
            fsm_command->motor_command.kd[i] = rl.params.Get<std::vector<float>>("rl_kd")[i];
            fsm_command->motor_command.tau[i] = 0;
        }
    }
}
