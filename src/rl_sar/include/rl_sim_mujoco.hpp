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
#include <array>
#include <chrono>
#include <mutex>
#include <thread>

#include <mujoco/mujoco.h>
#include "joystick.hh"
#include "mujoco_utils.hpp"

#ifdef USE_MUJOCO_ROS2
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#endif

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
#ifdef USE_MUJOCO_ROS2
    void StartRosInterface();
    void PublishRosArmState();
    void PublishRosArmTarget();
    void PublishRosOdometry();
    void RosArmCommandCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);
    void RosArmModeCallback(const std_msgs::msg::String::SharedPtr msg);
    void RosBaseCommandCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void ApplyRosBaseCommand();
    std::shared_ptr<rclcpp::Node> ros_node_;
    std::thread ros_thread_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr ros_arm_state_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ros_arm_mode_pub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr ros_arm_target_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ros_odom_pub_;
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr ros_arm_command_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ros_arm_mode_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr ros_base_command_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ros_fsm_key_sub_;
    std::mutex ros_arm_mutex_;
    std::vector<float> ros_arm_target_q_ = std::vector<float>(6, 0.0f);
    std::vector<float> ros_arm_target_dq_ = std::vector<float>(6, 0.0f);
    bool ros_arm_target_valid_ = false;
    bool ros_arm_hold_valid_ = false;
    // [vx, vy, wz, height, pitch, roll] from /go2_x5/base/command (WBC mode)
    std::array<float, 6> ros_base_command_{};
    std::chrono::steady_clock::time_point ros_base_command_time_;
    bool ros_base_command_seen_ = false;
    bool ros_base_driven_ = false;
    std::vector<float> ros_arm_hold_q_ = std::vector<float>(6, 0.0f);
    std::string ros_arm_mode_ = "HOLD";
#endif
};

#endif // RL_SIM_HPP
