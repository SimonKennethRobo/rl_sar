/*
 * Copyright (c) 2024-2026 Ziqi Fan and contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rl_real_go2_ros2.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <csignal>
#include <cmath>
#include <cctype>
#include <cstring>
#include <iostream>
#include <thread>

namespace
{
constexpr float kPosStop = 2.146e9F;
constexpr float kVelStop = 16000.0F;
constexpr int64_t kCheckModeApiId = 1001;
constexpr int64_t kReleaseModeApiId = 1003;
constexpr int32_t kMotionApiTimeout = -1;
volatile std::sig_atomic_t g_stop_requested = 0;

void RequestStop(int)
{
    g_stop_requested = 1;
}

template <typename Container>
bool AllFinite(const Container &values)
{
    return std::all_of(values.begin(), values.end(),
                       [](float value) { return std::isfinite(value); });
}

union KeySwitch
{
    struct
    {
        uint16_t R1 : 1;
        uint16_t L1 : 1;
        uint16_t start : 1;
        uint16_t select : 1;
        uint16_t R2 : 1;
        uint16_t L2 : 1;
        uint16_t F1 : 1;
        uint16_t F2 : 1;
        uint16_t A : 1;
        uint16_t B : 1;
        uint16_t X : 1;
        uint16_t Y : 1;
        uint16_t up : 1;
        uint16_t right : 1;
        uint16_t down : 1;
        uint16_t left : 1;
    } components;
    uint16_t value;
};

struct RawBmsCmd
{
    uint8_t off;
    std::array<uint8_t, 3> reserve;
};

struct RawMotorCmd
{
    uint8_t mode;
    float q;
    float dq;
    float tau;
    float kp;
    float kd;
    std::array<uint32_t, 3> reserve;
};

struct RawLowCmd
{
    std::array<uint8_t, 2> head;
    uint8_t level_flag;
    uint8_t frame_reserve;
    std::array<uint32_t, 2> sn;
    std::array<uint32_t, 2> version;
    uint16_t bandwidth;
    std::array<RawMotorCmd, 20> motor_cmd;
    RawBmsCmd bms_cmd;
    std::array<uint8_t, 40> wireless_remote;
    std::array<uint8_t, 12> led;
    std::array<uint8_t, 2> fan;
    uint8_t gpio;
    uint32_t reserve;
    uint32_t crc;
};

uint32_t Crc32Core(uint32_t *ptr, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFF;
    constexpr uint32_t polynomial = 0x04c11db7;
    for (uint32_t i = 0; i < len; ++i)
    {
        uint32_t xbit = 1U << 31;
        const uint32_t data = ptr[i];
        for (uint32_t bits = 0; bits < 32; ++bits)
        {
            crc = (crc & 0x80000000U) ? (crc << 1U) ^ polynomial : crc << 1U;
            if (data & xbit) crc ^= polynomial;
            xbit >>= 1U;
        }
    }
    return crc;
}

int64_t MonotonicNanoseconds()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}
}  // namespace

RLRealGo2Ros2::RLRealGo2Ros2(int argc, char **argv)
    : rclcpp::Node("rl_real_go2_ros2")
{
    const std::string mode = argc > 1 ? argv[1] : "";
    if (!mode.empty() && mode != "wheel" && mode != "x5")
    {
        throw std::runtime_error("Usage: rl_real_go2_ros2 [wheel|x5]");
    }
    x5_mode_ = mode == "x5";
    ang_vel_axis = "body";
    robot_name = x5_mode_ ? "go2_x5" : (mode == "wheel" ? "go2w" : "go2");
    ReadYaml(robot_name, "base.yaml");

    if (!FSMManager::GetInstance().IsTypeSupported(robot_name))
    {
        throw std::runtime_error("No FSM registered for robot: " + robot_name);
    }
    // CreateFSM accepts an untyped context and its factories cast it back to
    // RL*. Pass the adjusted base pointer explicitly: RL is the second base of
    // this class, so passing the most-derived `this` through void* would point
    // the FSM at the rclcpp::Node subobject instead.
    auto fsm_ptr = FSMManager::GetInstance().CreateFSM(
        robot_name, static_cast<RL *>(this));
    if (!fsm_ptr)
    {
        throw std::runtime_error("Failed to create FSM for robot: " + robot_name);
    }
    fsm = *fsm_ptr;

    InitJointNum(params.Get<int>("num_of_dofs"));
    InitOutputs();
    InitControl();

    lowcmd_publisher_ = create_publisher<unitree_go::msg::LowCmd>("/lowcmd", rclcpp::QoS(10));
    lowstate_subscriber_ = create_subscription<unitree_go::msg::LowState>(
        "/lowstate", rclcpp::QoS(1),
        [this](const unitree_go::msg::LowState::SharedPtr msg) { LowStateCallback(msg); });
    joystick_subscriber_ = create_subscription<unitree_go::msg::WirelessController>(
        "/wirelesscontroller", rclcpp::QoS(1),
        [this](const unitree_go::msg::WirelessController::SharedPtr msg) { JoystickCallback(msg); });
    cmd_vel_subscriber_ = create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", rclcpp::SystemDefaultsQoS(),
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) { CmdvelCallback(msg); });
    if (x5_mode_)
    {
        odometry_subscriber_ = create_subscription<nav_msgs::msg::Odometry>(
            "/go2_x5/slam/odometry", rclcpp::SensorDataQoS(),
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) { OdometryCallback(msg); });
        arm_state_subscriber_ = create_subscription<sensor_msgs::msg::JointState>(
            "/go2_x5/arm/state", rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::JointState::SharedPtr msg) { ArmStateCallback(msg); });
        arm_command_subscriber_ = create_subscription<trajectory_msgs::msg::JointTrajectory>(
            "/go2_x5/arm/command/request", rclcpp::QoS(10),
            [this](const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) { ArmCommandCallback(msg); });
        arm_mode_subscriber_ = create_subscription<std_msgs::msg::String>(
            "/go2_x5/arm/mode/request", rclcpp::QoS(10),
            [this](const std_msgs::msg::String::SharedPtr msg) { ArmModeCallback(msg); });
        fsm_key_subscriber_ = create_subscription<std_msgs::msg::String>(
            "/go2_x5/fsm/key", rclcpp::QoS(10),
            [this](const std_msgs::msg::String::SharedPtr msg) { InjectKey(msg->data); });
        base_command_subscriber_ = create_subscription<std_msgs::msg::Float32MultiArray>(
            "/go2_x5/base/command", rclcpp::QoS(10),
            [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) { BaseCommandCallback(msg); });
        arm_command_publisher_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/go2_x5/arm/command/target", rclcpp::QoS(10));
        arm_mode_target_publisher_ = create_publisher<std_msgs::msg::String>(
            "/go2_x5/arm/mode/target", rclcpp::QoS(10));
        arm_mode_state_publisher_ = create_publisher<std_msgs::msg::String>(
            "/go2_x5/arm/mode/state", rclcpp::QoS(1).transient_local());
        PublishArmMode(arm_mode_);
    }

    motion_request_publisher_ = create_publisher<unitree_api::msg::Request>(
        "/api/motion_switcher/request", rclcpp::QoS(1));
    motion_response_subscriber_ = create_subscription<unitree_api::msg::Response>(
        "/api/motion_switcher/response", rclcpp::QoS(1),
        [this](const unitree_api::msg::Response::SharedPtr msg) { MotionResponseCallback(msg); });
}

RLRealGo2Ros2::~RLRealGo2Ros2()
{
    Stop();
}

bool RLRealGo2Ros2::Start()
{
    if (!WaitForLowState(std::chrono::seconds(5)))
    {
        RCLCPP_ERROR(get_logger(), "No /lowstate received within 5 seconds; control loops will not start");
        return false;
    }
    if (!DeactivateMotionService())
    {
        RCLCPP_ERROR(get_logger(), "Failed to deactivate the motion service; control loops will not start");
        return false;
    }

    loop_keyboard_ = std::make_shared<LoopFunc>(
        "loop_keyboard", 0.05, std::bind(&RLRealGo2Ros2::KeyboardInterface, this));
    loop_control_ = std::make_shared<LoopFunc>(
        "loop_control", params.Get<float>("dt"), std::bind(&RLRealGo2Ros2::RobotControl, this));
    loop_rl_ = std::make_shared<LoopFunc>(
        "loop_rl", params.Get<float>("dt") * params.Get<int>("decimation"),
        std::bind(&RLRealGo2Ros2::RunModel, this));
    started_ = true;
    loop_keyboard_->start();
    loop_control_->start();
    loop_rl_->start();
    return true;
}

void RLRealGo2Ros2::Stop()
{
    if (!started_.exchange(false)) return;
    if (loop_keyboard_) loop_keyboard_->shutdown();
    if (loop_control_) loop_control_->shutdown();
    if (loop_rl_) loop_rl_->shutdown();
    std::cout << LOGGER::INFO << "RLRealGo2Ros2 exit" << std::endl;
}

bool RLRealGo2Ros2::HasLoopFailure(std::string *message) const
{
    const std::array<std::shared_ptr<LoopFunc>, 3> loops = {
        loop_keyboard_, loop_control_, loop_rl_};
    for (const auto &loop : loops)
    {
        if (loop && loop->failed())
        {
            if (message) *message = loop->errorMessage();
            return true;
        }
    }
    return false;
}

void RLRealGo2Ros2::GetState(RobotState<float> *state)
{
    unitree_go::msg::LowState low_state;
    unitree_go::msg::WirelessController joystick;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        low_state = low_state_;
        joystick = joystick_;
    }

    KeySwitch keys{};
    keys.value = joystick.keys;
    if (keys.components.A) control.SetGamepad(Input::Gamepad::A);
    if (keys.components.B) control.SetGamepad(Input::Gamepad::B);
    if (keys.components.X) control.SetGamepad(Input::Gamepad::X);
    if (keys.components.Y) control.SetGamepad(Input::Gamepad::Y);
    if (keys.components.L1) control.SetGamepad(Input::Gamepad::LB);
    if (keys.components.R1) control.SetGamepad(Input::Gamepad::RB);
    if (keys.components.F1) control.SetGamepad(Input::Gamepad::LStick);
    if (keys.components.F2) control.SetGamepad(Input::Gamepad::RStick);
    if (keys.components.up) control.SetGamepad(Input::Gamepad::DPadUp);
    if (keys.components.down) control.SetGamepad(Input::Gamepad::DPadDown);
    if (keys.components.left) control.SetGamepad(Input::Gamepad::DPadLeft);
    if (keys.components.right) control.SetGamepad(Input::Gamepad::DPadRight);
    if (keys.components.L1 && keys.components.A) control.SetGamepad(Input::Gamepad::LB_A);
    if (keys.components.L1 && keys.components.B) control.SetGamepad(Input::Gamepad::LB_B);
    if (keys.components.L1 && keys.components.X) control.SetGamepad(Input::Gamepad::LB_X);
    if (keys.components.L1 && keys.components.Y) control.SetGamepad(Input::Gamepad::LB_Y);
    if (keys.components.L1 && keys.components.F1) control.SetGamepad(Input::Gamepad::LB_LStick);
    if (keys.components.L1 && keys.components.F2) control.SetGamepad(Input::Gamepad::LB_RStick);
    if (keys.components.L1 && keys.components.up) control.SetGamepad(Input::Gamepad::LB_DPadUp);
    if (keys.components.L1 && keys.components.down) control.SetGamepad(Input::Gamepad::LB_DPadDown);
    if (keys.components.L1 && keys.components.left) control.SetGamepad(Input::Gamepad::LB_DPadLeft);
    if (keys.components.L1 && keys.components.right) control.SetGamepad(Input::Gamepad::LB_DPadRight);
    if (keys.components.R1 && keys.components.A) control.SetGamepad(Input::Gamepad::RB_A);
    if (keys.components.R1 && keys.components.B) control.SetGamepad(Input::Gamepad::RB_B);
    if (keys.components.R1 && keys.components.X) control.SetGamepad(Input::Gamepad::RB_X);
    if (keys.components.R1 && keys.components.Y) control.SetGamepad(Input::Gamepad::RB_Y);
    if (keys.components.R1 && keys.components.F1) control.SetGamepad(Input::Gamepad::RB_LStick);
    if (keys.components.R1 && keys.components.F2) control.SetGamepad(Input::Gamepad::RB_RStick);
    if (keys.components.R1 && keys.components.up) control.SetGamepad(Input::Gamepad::RB_DPadUp);
    if (keys.components.R1 && keys.components.down) control.SetGamepad(Input::Gamepad::RB_DPadDown);
    if (keys.components.R1 && keys.components.left) control.SetGamepad(Input::Gamepad::RB_DPadLeft);
    if (keys.components.R1 && keys.components.right) control.SetGamepad(Input::Gamepad::RB_DPadRight);
    if (keys.components.L1 && keys.components.R1) control.SetGamepad(Input::Gamepad::LB_RB);

    control.x = joystick.ly;
    control.y = -joystick.lx;
    control.yaw = -joystick.rx;

    for (int i = 0; i < 4; ++i) state->imu.quaternion[i] = low_state.imu_state.quaternion[i];
    for (int i = 0; i < 3; ++i) state->imu.gyroscope[i] = low_state.imu_state.gyroscope[i];

    const int robot_dofs = x5_mode_ ? params.Get<int>("num_leg_dofs") : params.Get<int>("num_of_dofs");
    const auto joint_mapping = params.Get<std::vector<int>>("joint_mapping");
    for (int i = 0; i < robot_dofs; ++i)
    {
        const auto &motor = low_state.motor_state[joint_mapping[i]];
        state->motor_state.q[i] = motor.q;
        state->motor_state.dq[i] = motor.dq;
        state->motor_state.tau_est[i] = motor.tau_est;
    }
    if (x5_mode_) CopyExternalObservations(state);
}

void RLRealGo2Ros2::SetCommand(const RobotCommand<float> *command)
{
    unitree_go::msg::LowCmd low_cmd{};
    low_cmd.head[0] = 0xFE;
    low_cmd.head[1] = 0xEF;
    low_cmd.level_flag = 0xFF;
    for (auto &motor : low_cmd.motor_cmd)
    {
        motor.mode = 0x01;
        motor.q = kPosStop;
        motor.dq = kVelStop;
    }

    const int commanded_dofs = x5_mode_ ? params.Get<int>("num_leg_dofs") : params.Get<int>("num_of_dofs");
    const auto joint_mapping = params.Get<std::vector<int>>("joint_mapping");
    for (int i = 0; i < commanded_dofs; ++i)
    {
        auto &motor = low_cmd.motor_cmd[joint_mapping[i]];
        motor.mode = 0x01;
        motor.q = command->motor_command.q[i];
        motor.dq = command->motor_command.dq[i];
        motor.kp = command->motor_command.kp[i];
        motor.kd = command->motor_command.kd[i];
        motor.tau = command->motor_command.tau[i];
    }
    SetLowCmdCrc(low_cmd);
    lowcmd_publisher_->publish(low_cmd);
}

void RLRealGo2Ros2::RobotControl()
{
    GetState(&robot_state);
    if (x5_mode_)
    {
        UpdateArmModeFromInput();
        CheckArmCommandWatchdog();
    }
    StateController(&robot_state, &robot_command);
    if (x5_mode_)
    {
        // The leg FSM's passive state is Go2 damping. It is a safety boundary:
        // force the arm through the same canonical mode path so the ARX SDK
        // cannot keep tracking an OCS2/HOLD target while the base is limp.
        const bool base_damping = fsm.current_state_ &&
            fsm.current_state_->GetStateName() == "RLFSMStatePassive";
        if (base_damping)
        {
            bool arm_already_damping = false;
            {
                std::lock_guard<std::mutex> lock(external_obs_mutex_);
                arm_already_damping = arm_mode_ == "DAMPING";
            }
            if (!arm_already_damping)
            {
                std_msgs::msg::String damping;
                damping.data = "DAMPING";
                ArmModeCallback(std::make_shared<std_msgs::msg::String>(damping));
            }
        }
        {
            std::lock_guard<std::mutex> lock(external_obs_mutex_);
            if (!have_arm_state_) {
                control.ClearInput();
                return;
            }
        }
        ApplyArmMode(robot_state, &robot_command);
        PublishArmCommand(robot_command);
        ApplyBaseCommand();
    }
    control.ClearInput();
    SetCommand(&robot_command);
}

void RLRealGo2Ros2::RunModel()
{
    if (!rl_init_done) return;
    if (x5_mode_ && !CopyExternalObservations()) return;

    episode_length_buf += 1;
    obs.ang_vel = robot_state.imu.gyroscope;
    obs.commands = {control.x, control.y, control.yaw};
    if (control.navigation_mode)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        obs.commands = {static_cast<float>(cmd_vel_.linear.x),
                        static_cast<float>(cmd_vel_.linear.y),
                        static_cast<float>(cmd_vel_.angular.z)};
    }
    obs.base_quat = robot_state.imu.quaternion;
    obs.dof_pos = robot_state.motor_state.q;
    obs.dof_vel = robot_state.motor_state.dq;
    obs.actions = Forward();

    if (x5_mode_)
    {
        const int leg_dofs = params.Get<int>("num_leg_dofs");
        if (static_cast<int>(obs.actions.size()) != leg_dofs)
        {
            std::cout << LOGGER::ERROR << "[Go2-X5] policy returned " << obs.actions.size()
                      << " actions, expected " << leg_dofs << std::endl;
            return;
        }
        obs.actions.resize(params.Get<int>("num_of_dofs"), 0.0F);
    }

    ComputeOutput(obs.actions, output_dof_pos, output_dof_vel, output_dof_tau);
    if (!output_dof_pos.empty()) output_dof_pos_queue.push(output_dof_pos);
    if (!output_dof_vel.empty()) output_dof_vel_queue.push(output_dof_vel);
    if (!output_dof_tau.empty()) output_dof_tau_queue.push(output_dof_tau);
    TorqueProtect(output_dof_tau);
}

std::vector<float> RLRealGo2Ros2::Forward()
{
    std::unique_lock<std::mutex> lock(model_mutex, std::try_to_lock);
    if (!lock.owns_lock()) return obs.actions;

    const std::vector<float> clamped_obs = ComputeObservation();
    std::vector<float> actions;
    if (!params.Get<std::vector<int>>("observations_history").empty())
    {
        history_obs_buf.insert(clamped_obs);
        history_obs = history_obs_buf.get_obs_vec(params.Get<std::vector<int>>("observations_history"));
        actions = model->forward({history_obs});
    }
    else
    {
        actions = model->forward({clamped_obs});
    }

    const auto upper = params.Get<std::vector<float>>("clip_actions_upper");
    const auto lower = params.Get<std::vector<float>>("clip_actions_lower");
    return (!upper.empty() && !lower.empty()) ? clamp(actions, lower, upper) : actions;
}

void RLRealGo2Ros2::LowStateCallback(const unitree_go::msg::LowState::SharedPtr msg)
{
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        low_state_ = *msg;
        have_low_state_ = true;
    }
    lowstate_cv_.notify_all();
}

void RLRealGo2Ros2::JoystickCallback(const unitree_go::msg::WirelessController::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    joystick_ = *msg;
}

void RLRealGo2Ros2::CmdvelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    cmd_vel_ = *msg;
}

void RLRealGo2Ros2::OdometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    std::vector<float> lin_vel = {
        static_cast<float>(msg->twist.twist.linear.x),
        static_cast<float>(msg->twist.twist.linear.y),
        static_cast<float>(msg->twist.twist.linear.z)};
    const float base_height = static_cast<float>(msg->pose.pose.position.z);
    std::vector<float> quaternion = {
        static_cast<float>(msg->pose.pose.orientation.w),
        static_cast<float>(msg->pose.pose.orientation.x),
        static_cast<float>(msg->pose.pose.orientation.y),
        static_cast<float>(msg->pose.pose.orientation.z)};
    if (!AllFinite(lin_vel) || !std::isfinite(base_height) || !AllFinite(quaternion))
    {
        WarnExternalObservations("odometry contains non-finite values");
        return;
    }
    const float norm = std::sqrt(
        quaternion[0] * quaternion[0] + quaternion[1] * quaternion[1] +
        quaternion[2] * quaternion[2] + quaternion[3] * quaternion[3]);
    if (norm < 1.0e-6F)
    {
        WarnExternalObservations("odometry orientation quaternion has zero norm");
        return;
    }
    for (float &value : quaternion) value /= norm;
    const auto euler = QuaternionToEuler(quaternion);

    std::lock_guard<std::mutex> lock(external_obs_mutex_);
    external_obs_.lin_vel = std::move(lin_vel);
    external_obs_.base_height = base_height;
    external_obs_.body_pose = {euler[1], euler[0]};
    external_obs_.have_odometry = true;
    external_obs_.odometry_stamp = std::chrono::steady_clock::now();
}

void RLRealGo2Ros2::ArmStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(external_obs_mutex_);
    if (msg->position.size() < 6) return;
    for (int i = 0; i < 6; ++i)
    {
        const std::string expected = "x5_joint" + std::to_string(i + 1);
        auto it = std::find(msg->name.begin(), msg->name.end(), expected);
        const size_t index = msg->name.empty() ? static_cast<size_t>(i) :
            (it == msg->name.end() ? static_cast<size_t>(i) : static_cast<size_t>(it - msg->name.begin()));
        if (index >= msg->position.size()) return;
        arm_q_[i] = static_cast<float>(msg->position[index]);
        arm_dq_[i] = index < msg->velocity.size() ? static_cast<float>(msg->velocity[index]) : 0.0F;
    }
    have_arm_state_ = true;
    if (arm_mode_ == "HOLD" && arm_hold_q_.size() == arm_q_.size() && last_published_arm_mode_.empty())
    {
        arm_hold_q_ = arm_q_;
    }
}

void RLRealGo2Ros2::ArmCommandCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{
    if (msg->points.empty() || msg->points.back().positions.size() != 6) return;
    if (!msg->joint_names.empty() && msg->joint_names.size() == 6) {
        for (int i = 0; i < 6; ++i) {
            if (msg->joint_names[i] != "x5_joint" + std::to_string(i + 1)) {
                RCLCPP_WARN(get_logger(), "Ignoring arm request with joint order mismatch at %d", i);
                return;
            }
        }
    }
    std::vector<float> q(6), dq(6, 0.0F);
    const auto &point = msg->points.back();
    for (int i = 0; i < 6; ++i)
    {
        q[i] = static_cast<float>(point.positions[i]);
        if (i < static_cast<int>(point.velocities.size())) dq[i] = static_cast<float>(point.velocities[i]);
    }
    std::string mode;
    {
        std::lock_guard<std::mutex> lock(external_obs_mutex_);
        mode = arm_mode_;
        arm_command_time_ = std::chrono::steady_clock::now();
        arm_command_seen_ = true;
        // Buffer the newest request even if the mode callback is delivered a
        // few DDS scheduling cycles later; the mode transition will consume it.
        arm_command_q_ = q;
        arm_command_dq_ = dq;
    }
    if (mode == "OCS2" || mode == "WBC") SetExternalArmTarget(q, dq);
}

void RLRealGo2Ros2::ArmModeCallback(const std_msgs::msg::String::SharedPtr msg)
{
    std::string mode = msg->data;
    std::transform(mode.begin(), mode.end(), mode.begin(),
                   [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
    if (mode != "HOME" && mode != "HOLD" && mode != "DAMPING" && mode != "OCS2" && mode != "WBC")
    {
        RCLCPP_WARN(get_logger(), "Ignoring unknown arm mode '%s'", msg->data.c_str());
        return;
    }
    {
        std::lock_guard<std::mutex> lock(external_obs_mutex_);
        if (mode == "HOLD" && have_arm_state_) arm_hold_q_ = arm_q_;
        if (mode == "OCS2" || mode == "WBC")
        {
            if (have_arm_state_ && arm_command_q_.size() == arm_q_.size())
            {
                arm_command_q_ = arm_q_;
                arm_command_dq_.assign(arm_q_.size(), 0.0F);
            }
            arm_command_seen_ = false;
            arm_command_time_ = std::chrono::steady_clock::now();
        }
        arm_mode_ = mode;
        arm_mode_display = mode;
    }
    if (mode == "DAMPING") ClearExternalArmTarget();
    PublishArmMode(mode);
    if (mode == "HOLD" && fsm.current_state_ &&
        fsm.current_state_->GetStateName() == "RLFSMStateOCS2Manip")
    {
        // HOLD returns base authority to the RL locomotion/remote-control FSM.
        fsm.RequestStateChange("RLFSMStateRLLocomotion");
    }
#ifdef USE_OCS2_BRIDGE
    // A mode request may come from ROS without a simultaneous FSM key. WBC
    // therefore also enters the bridge-backed state here.
    if ((mode == "OCS2" || mode == "WBC") && fsm.current_state_ &&
        fsm.current_state_->GetStateName() == "RLFSMStateRLLocomotion")
    {
        fsm.RequestStateChange("RLFSMStateOCS2Manip");
    }
#endif
}

void RLRealGo2Ros2::BaseCommandCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    if (msg->data.size() != 6) return;
    std::lock_guard<std::mutex> lock(external_obs_mutex_);
    std::copy(msg->data.begin(), msg->data.end(), base_command_.begin());
    base_command_time_ = std::chrono::steady_clock::now();
    base_command_seen_ = true;
}

void RLRealGo2Ros2::CheckArmCommandWatchdog()
{
    std::string mode;
    bool seen = false;
    std::chrono::steady_clock::time_point received;
    {
        std::lock_guard<std::mutex> lock(external_obs_mutex_);
        mode = arm_mode_;
        seen = arm_command_seen_;
        received = arm_command_time_;
    }
    if (mode != "OCS2" && mode != "WBC") return;
    constexpr auto kArmCommandTimeout = std::chrono::milliseconds(500);
    const bool stale = !seen ||
        std::chrono::steady_clock::now() - received > kArmCommandTimeout;
    if (!stale) return;

    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Arm MPC command watchdog expired; switching to HOLD");
    std_msgs::msg::String hold;
    hold.data = "HOLD";
    ArmModeCallback(std::make_shared<std_msgs::msg::String>(hold));
}

// WBC mode only: MPC base channels replace the operator's velocity/pose
// commands while the legs are in locomotion. Any other mode/state (including
// a stale link) stops or releases them, so leaving WBC always stops the base.
void RLRealGo2Ros2::ApplyBaseCommand()
{
    const bool locomotion = fsm.current_state_ &&
        fsm.current_state_->GetStateName() == "RLFSMStateRLLocomotion";
    std::array<float, 6> cmd{};
    bool wbc = false, fresh = false;
    {
        std::lock_guard<std::mutex> lock(external_obs_mutex_);
        wbc = arm_mode_ == "WBC";
        cmd = base_command_;
        fresh = base_command_seen_ &&
            std::chrono::steady_clock::now() - base_command_time_ < std::chrono::milliseconds(200);
    }
    if (wbc && locomotion)
    {
        if (fresh) ApplyExternalBaseCommand(cmd);
        else CoastExternalBaseCommand();
        base_driven_ = true;
    }
    else if (base_driven_)
    {
        ReleaseExternalBaseCommand();
        base_driven_ = false;
    }
}

void RLRealGo2Ros2::UpdateArmModeFromInput()
{
    std::string mode;
    if (control.current_keyboard == Input::Keyboard::Num2 || control.current_gamepad == Input::Gamepad::Y)
        mode = "HOLD";
    else if (control.current_keyboard == Input::Keyboard::Num3 || control.current_gamepad == Input::Gamepad::RB_DPadLeft)
        mode = "HOME";
    else if (control.current_keyboard == Input::Keyboard::Num4 || control.current_gamepad == Input::Gamepad::RB_DPadRight)
        mode = "OCS2";
    else if (control.current_keyboard == Input::Keyboard::Num5 || control.current_gamepad == Input::Gamepad::RB_DPadDown)
        mode = "WBC";
    else if (control.current_keyboard == Input::Keyboard::Num9 || control.current_gamepad == Input::Gamepad::B)
        mode = "DAMPING";
    else if (control.current_keyboard == Input::Keyboard::P || control.current_gamepad == Input::Gamepad::LB_X)
        mode = "DAMPING";
    if (!mode.empty())
    {
        std_msgs::msg::String msg;
        msg.data = mode;
        ArmModeCallback(std::make_shared<std_msgs::msg::String>(msg));

    }
}

void RLRealGo2Ros2::ApplyArmMode(const RobotState<float> &state, RobotCommand<float> *command)
{
    std::string mode;
    {
        std::lock_guard<std::mutex> lock(external_obs_mutex_);
        mode = arm_mode_;
    }
    const int begin = params.Get<int>("num_leg_dofs");
    const int dofs = params.Get<int>("num_arm_dofs", 6);
    const auto home = params.Get<std::vector<float>>("default_dof_pos");
    const auto kp = params.Get<std::vector<float>>("fixed_kp");
    const auto kd = params.Get<std::vector<float>>("fixed_kd");
    if (begin + dofs > static_cast<int>(command->motor_command.q.size())) return;
    std::vector<float> target(dofs, 0.0F), velocity(dofs, 0.0F);
    if (mode == "HOME")
    {
        // HOME is owned by arx5_ros2. ArmModeCallback() already publishes
        // HOME on /go2_x5/arm/mode/target, which calls reset_to_home() in the
        // ARX driver. Do not also stream a trajectory from rl_sar: that would
        // compete with the driver's native home controller.
        for (int i = 0; i < dofs; ++i)
        {
            command->motor_command.q[begin + i] = state.motor_state.q[begin + i];
            command->motor_command.dq[begin + i] = 0.0F;
            command->motor_command.kp[begin + i] = 0.0F;
            command->motor_command.kd[begin + i] = 0.0F;
            command->motor_command.tau[begin + i] = 0.0F;
        }
        ClearExternalArmTarget();
        return;
    }
    else if (mode == "HOLD")
    {
        // HOLD is owned by arx5_ros2. Its mode callback latches the measured
        // joint state and enables the SDK joint controller with its own gains.
        // Keep the Unitree-side arm command inert and do not stream YAML gains.
        for (int i = 0; i < dofs; ++i)
        {
            command->motor_command.q[begin + i] = state.motor_state.q[begin + i];
            command->motor_command.dq[begin + i] = 0.0F;
            command->motor_command.kp[begin + i] = 0.0F;
            command->motor_command.kd[begin + i] = 0.0F;
            command->motor_command.tau[begin + i] = 0.0F;
        }
        ClearExternalArmTarget();
        return;
    }
    else if (mode == "DAMPING")
    {
        for (int i = 0; i < dofs; ++i)
        {
            command->motor_command.q[begin + i] = state.motor_state.q[begin + i];
            command->motor_command.dq[begin + i] = 0.0F;
            command->motor_command.kp[begin + i] = 0.0F;
            command->motor_command.kd[begin + i] = 0.0F;
            command->motor_command.tau[begin + i] = 0.0F;
        }
        ClearExternalArmTarget();
        return;
    }
    else if (mode == "OCS2" || mode == "WBC")
    {
        std::lock_guard<std::mutex> lock(external_obs_mutex_);
        target = arm_command_q_;
        velocity = arm_command_dq_;
    }
    for (int i = 0; i < dofs; ++i)
    {
        command->motor_command.q[begin + i] = target[i];
        command->motor_command.dq[begin + i] = velocity[i];
        command->motor_command.kp[begin + i] = kp[begin + i];
        command->motor_command.kd[begin + i] = kd[begin + i];
        command->motor_command.tau[begin + i] = 0.0F;
    }
}

void RLRealGo2Ros2::PublishArmCommand(const RobotCommand<float> &command)
{
    std::string current_mode;
    { std::lock_guard<std::mutex> lock(external_obs_mutex_); current_mode = arm_mode_; }
    if (!arm_command_publisher_ || current_mode == "DAMPING" || current_mode == "HOME" || current_mode == "HOLD") return;
    trajectory_msgs::msg::JointTrajectory msg;
    msg.header.stamp = now();
    msg.joint_names.resize(6);
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.resize(6); point.velocities.resize(6);
    const int begin = params.Get<int>("num_leg_dofs");
    for (int i = 0; i < 6; ++i)
    {
        msg.joint_names[i] = "x5_joint" + std::to_string(i + 1);
        point.positions[i] = command.motor_command.q[begin + i];
        point.velocities[i] = command.motor_command.dq[begin + i];
    }
    point.time_from_start = rclcpp::Duration::from_seconds(std::max(0.01, static_cast<double>(params.Get<float>("dt"))));
    msg.points.push_back(point);
    arm_command_publisher_->publish(msg);
}

void RLRealGo2Ros2::PublishArmMode(const std::string &mode)
{
    if (!arm_mode_state_publisher_) return;
    std_msgs::msg::String msg; msg.data = mode;
    arm_mode_state_publisher_->publish(msg);
    if (arm_mode_target_publisher_)
    {
        // The ARX driver has no WBC concept: for it, WBC is just OCS2 tracking.
        std_msgs::msg::String driver_msg = msg;
        if (driver_msg.data == "WBC") driver_msg.data = "OCS2";
        arm_mode_target_publisher_->publish(driver_msg);
    }
    last_published_arm_mode_ = mode;
}

void RLRealGo2Ros2::MotionResponseCallback(const unitree_api::msg::Response::SharedPtr msg)
{
    {
        std::lock_guard<std::mutex> lock(motion_mutex_);
        if (msg->header.identity.id != pending_motion_request_id_) return;
        motion_response_ = *msg;
        motion_response_ready_ = true;
    }
    motion_cv_.notify_all();
}

bool RLRealGo2Ros2::WaitForLowState(std::chrono::seconds timeout)
{
    std::unique_lock<std::mutex> lock(state_mutex_);
    return lowstate_cv_.wait_for(lock, timeout, [this] { return have_low_state_; });
}

int32_t RLRealGo2Ros2::CallMotionApi(int64_t api_id, std::string *response_data)
{
    unitree_api::msg::Request request{};
    request.header.identity.id = MonotonicNanoseconds();
    request.header.identity.api_id = api_id;
    {
        std::lock_guard<std::mutex> lock(motion_mutex_);
        pending_motion_request_id_ = request.header.identity.id;
        motion_response_ready_ = false;
    }
    motion_request_publisher_->publish(request);

    std::unique_lock<std::mutex> lock(motion_mutex_);
    if (!motion_cv_.wait_for(lock, std::chrono::seconds(5),
                             [this] { return motion_response_ready_; }))
    {
        return kMotionApiTimeout;
    }
    if (response_data) *response_data = motion_response_.data;
    return motion_response_.header.status.code;
}

bool RLRealGo2Ros2::DeactivateMotionService()
{
    while (rclcpp::ok())
    {
        std::string response_data;
        const int32_t check_result = CallMotionApi(kCheckModeApiId, &response_data);
        if (check_result == kMotionApiTimeout)
        {
            RCLCPP_WARN(get_logger(),
                        "Motion switcher did not respond; continuing because /lowstate is available. "
                        "Make sure the built-in motion service is already disabled");
            return true;
        }
        if (check_result != 0)
        {
            RCLCPP_ERROR(get_logger(), "Motion CheckMode failed: %d", check_result);
            return false;
        }
        try
        {
            const auto response = nlohmann::json::parse(response_data);
            const std::string name = response.value("name", "");
            if (name.empty())
            {
                RCLCPP_INFO(get_logger(), "Motion control service is inactive");
                return true;
            }
            RCLCPP_WARN(get_logger(), "Releasing active motion mode: %s", name.c_str());
        }
        catch (const nlohmann::json::exception &error)
        {
            RCLCPP_ERROR(get_logger(), "Invalid motion switcher response: %s", error.what());
            return false;
        }
        const int32_t release_result = CallMotionApi(kReleaseModeApiId);
        if (release_result != 0)
        {
            RCLCPP_ERROR(get_logger(), "Motion ReleaseMode failed: %d", release_result);
            return false;
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    return false;
}

bool RLRealGo2Ros2::CopyExternalObservations(RobotState<float> *state)
{
    const auto now = std::chrono::steady_clock::now();
    std::unique_lock<std::mutex> lock(external_obs_mutex_);
    if (state)
    {
        const int begin = params.Get<int>("num_leg_dofs");
        const int end = begin + params.Get<int>("num_arm_dofs");
        if (!have_arm_state_ || static_cast<int>(arm_q_.size()) != end - begin)
        {
            std::fill(state->motor_state.q.begin() + begin, state->motor_state.q.begin() + end, 0.0F);
            std::fill(state->motor_state.dq.begin() + begin, state->motor_state.dq.begin() + end, 0.0F);
            std::fill(state->motor_state.tau_est.begin() + begin, state->motor_state.tau_est.begin() + end, 0.0F);
        }
        else
        {
            std::copy(arm_q_.begin(), arm_q_.end(), state->motor_state.q.begin() + begin);
            std::copy(arm_dq_.begin(), arm_dq_.end(), state->motor_state.dq.begin() + begin);
            std::fill(state->motor_state.tau_est.begin() + begin, state->motor_state.tau_est.begin() + end, 0.0F);
        }
        return true;
    }
    if (!external_obs_.have_odometry)
    {
        lock.unlock();
        WarnExternalObservations("waiting for odometry");
        return false;
    }
    const float age = std::chrono::duration<float>(now - external_obs_.odometry_stamp).count();
    if (age > params.Get<float>("external_obs_timeout", 0.2F))
    {
        lock.unlock();
        WarnExternalObservations("odometry exceeded external_obs_timeout");
        return false;
    }
    obs.lin_vel = external_obs_.lin_vel;
    obs.base_height = {external_obs_.base_height};
    obs.body_pose_actual = {
        external_obs_.base_height, external_obs_.body_pose[0], external_obs_.body_pose[1]};
    return true;
}

void RLRealGo2Ros2::WarnExternalObservations(const std::string &reason)
{
    const auto now = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock(external_obs_mutex_);
    if (last_external_obs_warning_.time_since_epoch().count() == 0 ||
        std::chrono::duration<float>(now - last_external_obs_warning_).count() >= 1.0F)
    {
        RCLCPP_WARN(get_logger(), "[Go2-X5 Obs] %s; policy inference paused", reason.c_str());
        last_external_obs_warning_ = now;
    }
}

void RLRealGo2Ros2::SetLowCmdCrc(unitree_go::msg::LowCmd &msg)
{
    RawLowCmd raw{};
    raw.head = msg.head;
    raw.level_flag = msg.level_flag;
    raw.frame_reserve = msg.frame_reserve;
    raw.sn = msg.sn;
    raw.version = msg.version;
    raw.bandwidth = msg.bandwidth;
    for (size_t i = 0; i < raw.motor_cmd.size(); ++i)
    {
        raw.motor_cmd[i].mode = msg.motor_cmd[i].mode;
        raw.motor_cmd[i].q = msg.motor_cmd[i].q;
        raw.motor_cmd[i].dq = msg.motor_cmd[i].dq;
        raw.motor_cmd[i].tau = msg.motor_cmd[i].tau;
        raw.motor_cmd[i].kp = msg.motor_cmd[i].kp;
        raw.motor_cmd[i].kd = msg.motor_cmd[i].kd;
        raw.motor_cmd[i].reserve = msg.motor_cmd[i].reserve;
    }
    raw.bms_cmd.off = msg.bms_cmd.off;
    raw.bms_cmd.reserve = msg.bms_cmd.reserve;
    raw.wireless_remote = msg.wireless_remote;
    raw.led = msg.led;
    raw.fan = msg.fan;
    raw.gpio = msg.gpio;
    raw.reserve = msg.reserve;
    raw.crc = Crc32Core(reinterpret_cast<uint32_t *>(&raw), (sizeof(raw) >> 2U) - 1U);
    msg.crc = raw.crc;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    // Foxy's default handler shuts the ROS context down before the low-level
    // publisher threads can be joined. Stop those threads first instead.
    rclcpp::uninstall_signal_handlers();
    std::signal(SIGINT, RequestStop);
    std::signal(SIGTERM, RequestStop);
    int result = 0;
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    std::shared_ptr<RLRealGo2Ros2> node;
    std::thread executor_thread;
    try
    {
        node = std::make_shared<RLRealGo2Ros2>(argc, argv);
        executor.add_node(node);
        executor_thread = std::thread([&executor] { executor.spin(); });

        if (!node->Start()) result = 1;
        while (result == 0 && rclcpp::ok() && !g_stop_requested)
        {
            std::string loop_error;
            if (node->HasLoopFailure(&loop_error))
            {
                RCLCPP_FATAL(node->get_logger(), "Control loop failed: %s", loop_error.c_str());
                result = 1;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        node->Stop();
    }
    catch (const std::exception &error)
    {
        RCLCPP_FATAL(rclcpp::get_logger("rl_real_go2_ros2"), "%s", error.what());
        result = 1;
    }
    if (node) node->Stop();
    executor.cancel();
    if (executor_thread.joinable()) executor_thread.join();
    if (node) executor.remove_node(node);
    rclcpp::shutdown();
    return result;
}
