/*
 * Copyright (c) 2024-2026 Ziqi Fan and contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef RL_REAL_GO2_ROS2_HPP
#define RL_REAL_GO2_ROS2_HPP

#include "fsm_go2.hpp"
#include "fsm_go2_x5.hpp"
#include "fsm_go2w.hpp"
#include "inference_runtime.hpp"
#include "loop.hpp"
#include "observation_buffer.hpp"
#include "rl_sdk.hpp"

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <unitree_api/msg/request.hpp>
#include <unitree_api/msg/response.hpp>
#include <unitree_go/msg/low_cmd.hpp>
#include <unitree_go/msg/low_state.hpp>
#include <unitree_go/msg/wireless_controller.hpp>

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <vector>

#if defined(USE_ROS2) && defined(ROS_DISTRO_FOXY)
namespace libstatistics_collector::topic_statistics_collector
{
template <>
struct TimeStamp<unitree_api::msg::Response>
{
    static std::pair<bool, int64_t> value(const unitree_api::msg::Response &)
    {
        return std::make_pair(true, 0);
    }
};
}  // namespace libstatistics_collector::topic_statistics_collector
#endif

class RLRealGo2Ros2 final : public rclcpp::Node, public RL
{
public:
    RLRealGo2Ros2(int argc, char **argv);
    ~RLRealGo2Ros2() override;

    bool Start();
    void Stop();
    bool HasLoopFailure(std::string *message = nullptr) const;

private:
    using SteadyTime = std::chrono::steady_clock::time_point;

    struct ExternalObservationState
    {
        std::vector<float> lin_vel = std::vector<float>(3, 0.0f);
        float base_height = 0.0f;
        std::vector<float> body_pose = std::vector<float>(2, 0.0f);
        bool have_odometry = false;
        SteadyTime odometry_stamp;
    };

    std::vector<float> Forward() override;
    void GetState(RobotState<float> *state) override;
    void SetCommand(const RobotCommand<float> *command) override;
    void RobotControl();
    void RunModel();

    void LowStateCallback(const unitree_go::msg::LowState::SharedPtr msg);
    void JoystickCallback(const unitree_go::msg::WirelessController::SharedPtr msg);
    void CmdvelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void OdometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void ArmStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void ArmCommandCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);
    void ArmModeCallback(const std_msgs::msg::String::SharedPtr msg);
    void MotionResponseCallback(const unitree_api::msg::Response::SharedPtr msg);
    void QueueFsmKey(const std::string &key);
    void ApplyNextFsmKey();

    bool WaitForLowState(std::chrono::seconds timeout);
    bool DeactivateMotionService();
    int32_t CallMotionApi(int64_t api_id, std::string *response_data = nullptr);
    bool CopyExternalObservations(RobotState<float> *state = nullptr);
    void UpdateArmModeFromInput();
    void ApplyArmMode(const RobotState<float> &state, RobotCommand<float> *command);
    void BaseCommandCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void ApplyBaseCommand();
    void CheckArmCommandWatchdog();
    void CheckOdometryWatchdog();
    void PublishArmCommand(const RobotCommand<float> &command);
    void PublishArmMode(const std::string &mode);
    void WarnExternalObservations(const std::string &reason);
    static void SetLowCmdCrc(unitree_go::msg::LowCmd &msg);

    bool x5_mode_ = false;
    std::atomic<bool> started_{false};

    std::shared_ptr<LoopFunc> loop_keyboard_;
    std::shared_ptr<LoopFunc> loop_control_;
    std::shared_ptr<LoopFunc> loop_rl_;

    rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr lowcmd_publisher_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr arm_command_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr arm_mode_target_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr arm_mode_state_publisher_;
    rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr lowstate_subscriber_;
    rclcpp::Subscription<unitree_go::msg::WirelessController>::SharedPtr joystick_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr arm_state_subscriber_;
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr arm_command_subscriber_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr arm_mode_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr base_command_subscriber_;
    // [phase_rad, rate_rad_s] of the policy gait clock, for the gait-aware MPC.
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gait_phase_publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr fsm_key_subscriber_;
    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr motion_request_publisher_;
    rclcpp::Subscription<unitree_api::msg::Response>::SharedPtr motion_response_subscriber_;

    std::mutex state_mutex_;
    std::condition_variable lowstate_cv_;
    unitree_go::msg::LowState low_state_{};
    unitree_go::msg::WirelessController joystick_{};
    geometry_msgs::msg::Twist cmd_vel_{};
    bool have_low_state_ = false;

    std::mutex fsm_key_mutex_;
    std::deque<std::string> pending_fsm_keys_;
    bool joystick_axes_active_ = false;
    bool joystick_pose_active_ = false;

    std::mutex motion_mutex_;
    std::condition_variable motion_cv_;
    int64_t pending_motion_request_id_ = 0;
    bool motion_response_ready_ = false;
    unitree_api::msg::Response motion_response_{};

    mutable std::mutex external_obs_mutex_;
    ExternalObservationState external_obs_;
    std::vector<float> arm_q_ = std::vector<float>(6, 0.0f);
    std::vector<float> arm_dq_ = std::vector<float>(6, 0.0f);
    std::vector<float> arm_hold_q_ = std::vector<float>(6, 0.0f);
    bool have_arm_state_ = false;
    std::vector<float> arm_command_q_ = std::vector<float>(6, 0.0f);
    std::vector<float> arm_command_dq_ = std::vector<float>(6, 0.0f);
    std::chrono::steady_clock::time_point arm_command_time_{};
    bool arm_command_seen_ = false;
    std::string arm_mode_ = "HOLD";
    // WBC mode: [vx, vy, wz, height, pitch, roll] from /go2_x5/base/command
    std::array<float, 6> base_command_{};
    std::chrono::steady_clock::time_point base_command_time_{};
    bool base_command_seen_ = false;
    bool base_driven_ = false;
    std::string last_published_arm_mode_;
    std::string last_input_arm_mode_;
    SteadyTime last_external_obs_warning_{};
    bool odometry_fault_latched_ = false;
};

#endif  // RL_REAL_GO2_ROS2_HPP
