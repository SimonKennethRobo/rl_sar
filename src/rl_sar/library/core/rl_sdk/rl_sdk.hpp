/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef RL_SDK_HPP
#define RL_SDK_HPP

#include <iostream>
#include <string>
#include <exception>
#include <unistd.h>
#include <algorithm>
#include <tbb/concurrent_queue.h>
#include <vector>
#include <memory>
#include <fstream>
#include <mutex>
#include <array>
#include <atomic>

#include <yaml-cpp/yaml.h>
#include "fsm.hpp"
#include "observation_buffer.hpp"
#include "vector_math.hpp"
#include "inference_runtime.hpp"
#include "logger.hpp"
#include "motion_loader.hpp"
#include "response_observation.hpp"

template <typename T>
struct RobotCommand
{
    struct MotorCommand
    {
        std::vector<int> mode;
        std::vector<T> q;
        std::vector<T> dq;
        std::vector<T> tau;
        std::vector<T> kp;
        std::vector<T> kd;

        void resize(size_t num_joints)
        {
            mode.resize(num_joints, 0);
            q.resize(num_joints, 0.0f);
            dq.resize(num_joints, 0.0f);
            tau.resize(num_joints, 0.0f);
            kp.resize(num_joints, 0.0f);
            kd.resize(num_joints, 0.0f);
        }
    } motor_command;
};

template <typename T>
struct RobotState
{
    struct IMU
    {
        std::vector<T> quaternion = {1.0f, 0.0f, 0.0f, 0.0f}; // w, x, y, z
        std::vector<T> gyroscope = {0.0f, 0.0f, 0.0f};
        std::vector<T> accelerometer = {0.0f, 0.0f, 0.0f};
    } imu;

    struct MotorState
    {
        std::vector<T> q;
        std::vector<T> dq;
        std::vector<T> ddq;
        std::vector<T> tau_est;
        std::vector<T> cur;

        void resize(size_t num_joints)
        {
            q.resize(num_joints, 0.0f);
            dq.resize(num_joints, 0.0f);
            ddq.resize(num_joints, 0.0f);
            tau_est.resize(num_joints, 0.0f);
            cur.resize(num_joints, 0.0f);
        }
    } motor_state;

    // Floating base state from an external estimator (MuJoCo ground truth in
    // sim, FAST-LIO / legged odometry on hardware). Stays all-zero for robots
    // whose policy does not observe it.
    struct Base
    {
        std::vector<T> lin_vel = {0.0f, 0.0f, 0.0f};  // BODY frame, m/s
        std::vector<T> position = {0.0f, 0.0f, 0.0f}; // WORLD frame, m
    } base;
};

namespace Input
{
    // Recommend: Num0-GetUp Num9-GetDown N-ToggleNavMode
    //            R-SimReset Enter-SimToggle
    //            M-MotorEnable K-MotorDisable P-MotorPassive
    //            Num1-BaseLocomotion Num2-Num8-Skills(7)
    //            WS-AxisX AD-AxisY QE-AxisYaw Space-AxisClear
    enum class Keyboard
    {
        None = 0,
        A, B, C, D, E, F, G, H, I, J, K, L, M,
        N, O, P, Q, R, S, T, U, V, W, X, Y, Z,
        Num0, Num1, Num2, Num3, Num4, Num5, Num6, Num7, Num8, Num9,
        Space, Enter, Escape,
        Up, Down, Left, Right
    };

    // Recommend: A-GetUp B-GetDown X-ToggleNavMode Y-None
    //            RB_Y-SimReset RB_X-SimToggle
    //            LB_A-MotorEnable LB_B-MotorDisable LB_X-MotorPassive
    //            RB_DPadUp-BaseLocomotion RB_DPadOthers/LB_DPadOthers-Skills(7)
    //            LY-AxisX LX-AxisY RX-AxisYaw
    enum class Gamepad
    {
        None = 0,
        A, B, X, Y, LB, RB, LStick, RStick, DPadUp, DPadDown, DPadLeft, DPadRight,
        LB_A, LB_B, LB_X, LB_Y, LB_LStick, LB_RStick, LB_DPadUp, LB_DPadDown, LB_DPadLeft, LB_DPadRight,
        RB_A, RB_B, RB_X, RB_Y, RB_LStick, RB_RStick, RB_DPadUp, RB_DPadDown, RB_DPadLeft, RB_DPadRight,
        LB_RB
    };
}

struct Control
{
    Input::Keyboard current_keyboard = Input::Keyboard::None, last_keyboard = Input::Keyboard::None;
    Input::Gamepad current_gamepad = Input::Gamepad::None, last_gamepad = Input::Gamepad::None;

    float x = 0.0f;
    float y = 0.0f;
    float yaw = 0.0f;
    // Body pose commands. Only consumed by policies that observe them
    // (e.g. RoboDuet's 6-wide dog command vector); ignored otherwise.
    float body_pitch = 0.0f;
    float body_roll = 0.0f;
    float body_height = 0.0f;
    bool navigation_mode = false;

    void SetKeyboard(Input::Keyboard keyboad)
    {
        if (current_keyboard != keyboad)
        {
            last_keyboard = current_keyboard;
            current_keyboard = keyboad;
        }
    }

    void SetGamepad(Input::Gamepad gamepad)
    {
        if (current_gamepad != gamepad)
        {
            last_gamepad = current_gamepad;
            current_gamepad = gamepad;
        }
    }

    void ClearInput()
    {
        current_keyboard = last_keyboard;
        current_gamepad = Input::Gamepad::None;
    }
};

struct YamlParams
{
    YAML::Node config_node;
    mutable std::mutex mutex;

    // Get config value by key
    // WARNING: For vectors/containers, store result in a variable before using iterators/references:
    //   ✓ auto vec = params.Get<std::vector<int>>("key"); vec.begin()
    //   ✗ params.Get<std::vector<int>>("key").begin()  // dangling reference!
    template<typename T>
    T Get(const std::string& key, const T& default_value = T()) const
    {
        std::lock_guard<std::mutex> lock(mutex);
        const YAML::Node value = config_node[key];
        if (value && value.IsDefined())
        {
            return value.as<T>();
        }
        return default_value;
    }

    bool Has(const std::string& key) const
    {
        std::lock_guard<std::mutex> lock(mutex);
        const YAML::Node value = config_node[key];
        return value && value.IsDefined();
    }

    void Set(const std::string& key, const YAML::Node& value)
    {
        std::lock_guard<std::mutex> lock(mutex);
        config_node[key] = YAML::Clone(value);
    }
};

template <typename T>
struct Observations
{
    std::vector<T> lin_vel;
    std::vector<T> ang_vel;
    std::vector<T> gravity_vec;
    std::vector<T> commands;
    std::vector<T> base_quat;
    std::vector<T> dof_pos;
    std::vector<T> dof_vel;
    std::vector<T> actions;
    std::vector<T> base_height; // single element, WORLD frame z of the base
    // Optional raw [height (m), pitch (rad), roll (rad)] supplied by an
    // external estimator. When empty, RoboDuet observations derive the same
    // values from base_height and base_quat.
    std::vector<T> body_pose_actual;
};

class RL
{
public:
    RL() {};
    ~RL() {};

    YamlParams params;
    Observations<float> obs;
    std::vector<int> obs_dims;

    RobotState<float> robot_state;
    RobotCommand<float> robot_command;
    tbb::concurrent_queue<std::vector<float>> output_dof_pos_queue;
    tbb::concurrent_queue<std::vector<float>> output_dof_vel_queue;
    tbb::concurrent_queue<std::vector<float>> output_dof_tau_queue;

    FSM fsm;
    RobotState<float> start_state;
    RobotState<float> now_state;
    bool rl_init_done = false;

    // init
    void InitObservations();
    void InitOutputs();
    void InitControl();
    void InitRL(std::string robot_config_path);
    // Runtime bundle entry point used by headless evaluators. The legacy
    // InitRL() remains unchanged and still resolves POLICY_DIR bundles.
    void InitRLFromBundle(const std::string& config_path, const std::string& model_path);
    void InitJointNum(size_t num_joints);

    // rl functions
    virtual std::vector<float> Forward() = 0;
    std::vector<float> ComputeObservation();
    void AdvanceServoObservation(const std::vector<float>& applied_command);
    virtual void GetState(RobotState<float> *state) = 0;
    virtual void SetCommand(const RobotCommand<float> *command) = 0;
    void StateController(const RobotState<float> *state, RobotCommand<float> *command);
    void ComputeOutput(const std::vector<float> &actions, std::vector<float> &output_dof_pos, std::vector<float> &output_dof_vel, std::vector<float> &output_dof_tau);

    // yaml params
    void ReadYaml(const std::string& file_path, const std::string& file_name);

    // csv logger
    std::string csv_filename;
    void CSVInit(std::string robot_name);
    void CSVLogger(const std::vector<float> &torque, const std::vector<float> &tau_est, const std::vector<float> &joint_pos, const std::vector<float> &joint_pos_target, const std::vector<float> &joint_vel);

    // control
    Control control;
    void KeyboardInterface();
    // Same key events as the terminal, from any source (e.g. the /go2_x5/fsm/key topic).
    // key: one character, or UP/DOWN/LEFT/RIGHT/ESC/ENTER/SPACE.
    void InjectKey(const std::string &key);
    bool PressKeyChar(int c);

    // history buffer
    ObservationBuffer history_obs_buf;
    std::vector<float> history_obs;

    // others
    int motiontime = 0;
    std::string robot_name, config_name;
    // Operator-facing arm mode shown alongside the locomotion FSM state.
    std::string arm_mode_display = "HOLD";
    bool simulation_running = true;
    std::string ang_vel_axis = "body";  // "world" or "body"
    unsigned long long episode_length_buf = 0;
    float motion_length = 0.0;
    int InverseJointMapping(int idx) const;

    // Gait clock phase accumulator, shared by policies that observe a periodic
    // gait signal (RoboDuet's clock_inputs). Must be reset to 0 whenever the RL
    // state is (re-)entered so the phase matches training's reset behaviour.
    float gait_indices = 0.0f;

    // Motion tracking (for mimic/dance tasks)
    std::unique_ptr<MotionLoader> motion_loader;

    // ---- External arm target ----
    // An off-policy planner (the OCS2 MPC bridge) driving the arm joints while
    // the locomotion policy keeps the legs. Those joints carry a zero
    // action_scale, so without this ComputeOutput would park them at
    // default_dof_pos forever. The targets are consumed verbatim: rate limiting
    // and joint-limit clamping belong to the producer, which knows its own rate.
    // Indices are policy order, i.e. the last num_arm_dofs entries.
    void SetExternalArmTarget(const std::vector<float> &q, const std::vector<float> &dq);
    void ClearExternalArmTarget();
    // True while a FSM or planner is supplying arm joint targets.
    bool HasExternalArmTarget() const;

    // Whole-body (WBC) mode: the MPC drives the locomotion command channels.
    // cmd = [vx, vy, wz, body_height, body_pitch, body_roll]. Values are clamped
    // to the active policy's limit_* ranges before being written to `control`.
    void ApplyExternalBaseCommand(const std::array<float, 6> &cmd);
    // Link went stale: bleed the velocity channels to zero, keep the pose channels.
    void CoastExternalBaseCommand();
    // Leaving WBC (or not in locomotion): zero every base channel.
    void ReleaseExternalBaseCommand();

    // protect func
    void TorqueProtect(const std::vector<float> &origin_output_dof_tau);
    void AttitudeProtect(const std::vector<float> &quaternion, float pitch_threshold, float roll_threshold);

    // rl module
    std::unique_ptr<InferenceRuntime::Model> model;
    // output buffer
    std::vector<float> output_dof_tau;
    std::vector<float> output_dof_pos;
    std::vector<float> output_dof_vel;

    // thread safety
    std::mutex model_mutex;

private:
    bool servo_advancing_ = false;
    unsigned long long servo_last_step_ = ~0ULL;
    std::vector<float> servo_clock_ = std::vector<float>(4, 0.0f);
    ResponseObservation response_observation_;
    bool response_enabled_ = false;
    bool response_initializing_ = false;
    unsigned long long response_last_step_ = 0;

    // Written by the bridge thread, read by the RL thread in ComputeOutput.
    mutable std::mutex external_arm_mutex_;
    std::vector<float> external_arm_q_;
    std::vector<float> external_arm_dq_;
    std::atomic<bool> use_external_arm_{false};

    /** Substitutes the external arm target into an in-flight ComputeOutput. */
    void ApplyExternalArmTarget(std::vector<float> &output_dof_pos,
                                std::vector<float> &output_dof_vel,
                                std::vector<float> &tau_target);
};

class RLFSMState : public FSMState
{
public:
    RLFSMState(RL& rl, const std::string& name)
        : FSMState(name), rl(rl), fsm_state(nullptr), fsm_command(nullptr) {}

    RL& rl;
    const RobotState<float> *fsm_state;
    RobotCommand<float> *fsm_command;

    bool Interpolate(
        float& percent,
        const std::vector<float>& start_pos,
        const std::vector<float>& target_pos,
        float duration_seconds,
        const std::string& description = "",
        bool use_fixed_gains = true
    );

    void RLControl();
};

#endif // RL_SDK_HPP
