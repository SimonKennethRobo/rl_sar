/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef RL_SIM_HPP
#define RL_SIM_HPP

// #define PLOT
// #define CSV_LOGGER

#include "rl_sdk.hpp"
#include "observation_buffer.hpp"
#include "inference_runtime.hpp"
#include "loop.hpp"
#include "fsm_all.hpp"

#include <csignal>
#include <vector>
#include <string>
#include <cstdlib>
#include <unistd.h>
#include <sys/wait.h>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <memory>

#include <mujoco/mujoco.h>
#include "joystick.hh"
#include "mujoco_utils.hpp"

#ifdef USE_JOYLINK
#include "joylink_client/joylink_client.h"
#include <map>
#endif

#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

class Button
{
public:
    Button() {}

    void update(bool state)
    {
        on_press = state ? state != pressed : false;
        on_release = state ? false : state != pressed;
        pressed = state;
    }

    bool pressed = false;
    bool on_press = false;
    bool on_release = false;
};

class RL_Sim : public RL
{
public:
    RL_Sim(int argc, char **argv);
    ~RL_Sim();

    std::unique_ptr<mj::Simulate> sim;
    static RL_Sim* instance;

private:
    // rl functions
    std::vector<float> Forward() override;
    void GetState(RobotState<float> *state) override;
    void SetCommand(const RobotCommand<float> *command) override;
    void RunModel();
    void RobotControl();
    // Command applied during the completed RC_s17 policy interval. Its
    // response/gait observation is advanced before the next inference.
    std::vector<float> completed_dog_command_ = std::vector<float>(11, 0.0f);

    // loop
    std::shared_ptr<LoopFunc> loop_keyboard;
    std::shared_ptr<LoopFunc> loop_joystick;
    std::shared_ptr<LoopFunc> loop_control;
    std::shared_ptr<LoopFunc> loop_rl;
    std::shared_ptr<LoopFunc> loop_plot;

    // plot
    const int plot_size = 100;
    std::vector<int> plot_t;
    std::vector<std::vector<float>> plot_real_joint_pos, plot_target_joint_pos;
    void Plot();

    // mujoco
    mjData *mj_data;
    mjModel *mj_model;
    std::string scene_name;

    // joystick
    std::unique_ptr<Joystick> sys_js;
    JoystickEvent sys_js_event;

    Button sys_js_button[20];
    int sys_js_axis[10] = {0};
    bool sys_js_active = false;
    bool sys_js_pose_active = false;
    float axis_deadzone = 0.05f;
    int sys_js_max_value = (1 << (16 - 1));
    void SetupSysJoystick(const std::string& device, int bits);
    void GetSysJoystick();

#ifdef USE_JOYLINK
    // Alternate gamepad input: reads a running `joylink` server over ZMQ
    // instead of /dev/input/js0 directly, so per-pad axis/button quirks are
    // fixed once in a YAML config rather than guessed in code. When enabled,
    // this replaces GetSysJoystick() as the joystick loop's target and follows
    // RoboDuet's own play_by_joy.py command layout (same axis roles, same
    // stance/gait step sizes), since that is the layout this policy was
    // designed to be driven with.
    std::unique_ptr<joylink_client::JoylinkClient> joylink;
    std::map<std::string, int> joylink_prev_buttons;
    float joylink_prev_dpad_x = 0.0f;
    float joylink_prev_dpad_y = 0.0f;
    bool joylink_defaults_captured = false;
    float joylink_default_gait_frequency = 0.0f;
    float joylink_default_stance_width = 0.0f;
    float joylink_default_stance_length = 0.0f;
    void SetupJoyLink(const std::string& config_path);
    void GetJoyLinkInput();
#endif

    // others
    std::string gazebo_model_name;
    std::map<std::string, float> joint_positions;
    std::map<std::string, float> joint_velocities;
    std::map<std::string, float> joint_efforts;
    void StartJointController(const std::string& ros_namespace, const std::vector<std::string>& names);
};

#endif // RL_SIM_HPP
