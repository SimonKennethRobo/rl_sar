/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rl_sim_mujoco.hpp"
#include <cctype>
#include <functional>

RL_Sim* RL_Sim::instance = nullptr;

RL_Sim::RL_Sim(int argc, char **argv)
{
    // Set static instance pointer early for signal handler
    instance = this;

    if (argc < 3)
    {
        std::cout << LOGGER::ERROR << "Usage: " << argv[0] << " robot_name scene_name" << std::endl;
        throw std::runtime_error("Invalid arguments");
    }
    else
    {
        this->robot_name = argv[1];
        this->scene_name = argv[2];
    }

    this->ang_vel_axis = "body";

    // now launch mujoco
    std::cout << LOGGER::INFO << "[MuJoCo] Launching..." << std::endl;

    // display an error if running on macOS under Rosetta 2
#if defined(__APPLE__) && defined(__AVX__)
    if (rosetta_error_msg)
    {
        DisplayErrorDialogBox("Rosetta 2 is not supported", rosetta_error_msg);
        std::exit(1);
    }
#endif

    // print version, check compatibility
    std::cout << LOGGER::INFO << "[MuJoCo] Version: " << mj_versionString() << std::endl;
    if (mjVERSION_HEADER != mj_version())
    {
        mju_error("Headers and library have different versions");
    }

    // scan for libraries in the plugin directory to load additional plugins
    scanPluginLibraries();

    mjvCamera cam;
    mjv_defaultCamera(&cam);

    mjvOption opt;
    mjv_defaultOption(&opt);

    mjvPerturb pert;
    mjv_defaultPerturb(&pert);

    // simulate object encapsulates the UI
    sim = std::make_unique<mj::Simulate>(
        std::make_unique<mj::GlfwAdapter>(),
        &cam, &opt, &pert, /* is_passive = */ false);

    std::string filename = std::string(CMAKE_CURRENT_SOURCE_DIR) + "/../rl_sar_zoo/" + this->robot_name + "_description/mjcf/" + this->scene_name + ".xml";

    // start physics thread
    std::thread physicsthreadhandle(&PhysicsThread, sim.get(), filename.c_str());
    physicsthreadhandle.detach();

    while (1)
    {
        if (d)
        {
            std::cout << LOGGER::INFO << "[MuJoCo] Data prepared" << std::endl;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    this->mj_model = m;
    this->mj_data = d;

    // Load startup settings before selecting the joystick device.
    this->ReadYaml(this->robot_name, "base.yaml");

#ifdef USE_JOYLINK
    {
        std::string joylink_config = (argc >= 4)
            ? std::string(argv[3])
            : std::string(CMAKE_CURRENT_SOURCE_DIR) + "/config/joylink_go2_x5.yaml";
        this->SetupJoyLink(joylink_config);
    }
#else
    this->SetupSysJoystick(this->params.Get<std::string>("joystick_device", "/dev/input/js0"), 16);
#endif

    // auto load FSM by robot_name
    if (FSMManager::GetInstance().IsTypeSupported(this->robot_name))
    {
        auto fsm_ptr = FSMManager::GetInstance().CreateFSM(this->robot_name, this);
        if (fsm_ptr)
        {
            this->fsm = *fsm_ptr;
        }
    }
    else
    {
        std::cout << LOGGER::ERROR << "[FSM] No FSM registered for robot: " << this->robot_name << std::endl;
    }

    // init robot
    this->InitJointNum(this->params.Get<int>("num_of_dofs"));
    this->InitOutputs();
    this->InitControl();
#ifdef USE_MUJOCO_ROS2
    StartRosInterface();
#endif

    // loop
    this->loop_control = std::make_shared<LoopFunc>("loop_control", this->params.Get<float>("dt"), std::bind(&RL_Sim::RobotControl, this));
    this->loop_rl = std::make_shared<LoopFunc>("loop_rl", this->params.Get<float>("dt") * this->params.Get<int>("decimation"), std::bind(&RL_Sim::RunModel, this));
    this->loop_control->start();
    this->loop_rl->start();

    // keyboard
    this->loop_keyboard = std::make_shared<LoopFunc>("loop_keyboard", 0.05, std::bind(&RL_Sim::KeyboardInterface, this));
    this->loop_keyboard->start();

    // joystick
#ifdef USE_JOYLINK
    this->loop_joystick = std::make_shared<LoopFunc>("loop_joystick", 0.01, std::bind(&RL_Sim::GetJoyLinkInput, this));
#else
    this->loop_joystick = std::make_shared<LoopFunc>("loop_joystick", 0.01, std::bind(&RL_Sim::GetSysJoystick, this));
#endif
    this->loop_joystick->start();

#ifdef PLOT
    this->plot_t = std::vector<int>(this->plot_size, 0);
    this->plot_real_joint_pos.resize(this->params.Get<int>("num_of_dofs"));
    this->plot_target_joint_pos.resize(this->params.Get<int>("num_of_dofs"));
    for (auto &vector : this->plot_real_joint_pos) { vector = std::vector<float>(this->plot_size, 0); }
    for (auto &vector : this->plot_target_joint_pos) { vector = std::vector<float>(this->plot_size, 0); }
    this->loop_plot = std::make_shared<LoopFunc>("loop_plot", 0.001, std::bind(&RL_Sim::Plot, this));
    this->loop_plot->start();
#endif
#ifdef CSV_LOGGER
    this->CSVInit(this->robot_name);
#endif

    std::cout << LOGGER::INFO << "RL_Sim start" << std::endl;

    // start simulation UI loop (blocking call)
    sim->RenderLoop();
}

RL_Sim::~RL_Sim()
{
    // Clear static instance pointer
    instance = nullptr;

    this->loop_keyboard->shutdown();
    this->loop_joystick->shutdown();
    this->loop_control->shutdown();
    this->loop_rl->shutdown();
#ifdef USE_MUJOCO_ROS2
    if (ros_node_) {
        if (rclcpp::ok()) rclcpp::shutdown();
        if (ros_thread_.joinable()) ros_thread_.join();
    }
#endif
#ifdef PLOT
    this->loop_plot->shutdown();
#endif
    std::cout << LOGGER::INFO << "RL_Sim exit" << std::endl;
}

void RL_Sim::GetState(RobotState<float> *state)
{
    if (mj_data)
    {
        state->imu.quaternion[0] = mj_data->sensordata[3 * this->params.Get<int>("num_of_dofs") + 0];
        state->imu.quaternion[1] = mj_data->sensordata[3 * this->params.Get<int>("num_of_dofs") + 1];
        state->imu.quaternion[2] = mj_data->sensordata[3 * this->params.Get<int>("num_of_dofs") + 2];
        state->imu.quaternion[3] = mj_data->sensordata[3 * this->params.Get<int>("num_of_dofs") + 3];

        state->imu.gyroscope[0] = mj_data->sensordata[3 * this->params.Get<int>("num_of_dofs") + 4];
        state->imu.gyroscope[1] = mj_data->sensordata[3 * this->params.Get<int>("num_of_dofs") + 5];
        state->imu.gyroscope[2] = mj_data->sensordata[3 * this->params.Get<int>("num_of_dofs") + 6];

        for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
        {
            state->motor_state.q[i] = mj_data->sensordata[this->params.Get<std::vector<int>>("joint_mapping")[i]];
            state->motor_state.dq[i] = mj_data->sensordata[this->params.Get<std::vector<int>>("joint_mapping")[i] + this->params.Get<int>("num_of_dofs")];
            state->motor_state.tau_est[i] = mj_data->sensordata[this->params.Get<std::vector<int>>("joint_mapping")[i] + 2 * this->params.Get<int>("num_of_dofs")];
        }

        // Optional floating-base ground truth. The rl_sar_zoo MJCF sensor block
        // is ordered jointpos(N), jointvel(N), jointactuatorfrc(N), framequat(4),
        // gyro(3), accelerometer(3), framepos(3, WORLD), framelinvel(3, WORLD),
        // so the base state starts at 3N + 10. Stands in for the state
        // estimator (FAST-LIO) used on hardware.
        if (this->params.Get<bool>("use_base_state_sensor", false))
        {
            const int base_sensor_offset = 3 * this->params.Get<int>("num_of_dofs") + 10;
            state->base.position[0] = mj_data->sensordata[base_sensor_offset + 0];
            state->base.position[1] = mj_data->sensordata[base_sensor_offset + 1];
            state->base.position[2] = mj_data->sensordata[base_sensor_offset + 2];
            std::vector<float> lin_vel_world = {
                (float)mj_data->sensordata[base_sensor_offset + 3],
                (float)mj_data->sensordata[base_sensor_offset + 4],
                (float)mj_data->sensordata[base_sensor_offset + 5]};
            // Training observes the base linear velocity in the BODY frame.
            state->base.lin_vel = QuatRotateInverse(state->imu.quaternion, lin_vel_world);
        }
    }
}

void RL_Sim::SetCommand(const RobotCommand<float> *command)
{
    if (mj_data)
    {
        for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
        {
            mj_data->ctrl[this->params.Get<std::vector<int>>("joint_mapping")[i]] =
                command->motor_command.tau[i] +
                command->motor_command.kp[i] * (command->motor_command.q[i] - mj_data->sensordata[this->params.Get<std::vector<int>>("joint_mapping")[i]]) +
                command->motor_command.kd[i] * (command->motor_command.dq[i] - mj_data->sensordata[this->params.Get<std::vector<int>>("joint_mapping")[i] + this->params.Get<int>("num_of_dofs")]);
        }
    }
}

void RL_Sim::RobotControl()
{
    // Lock the sim mutex once for the entire control cycle to prevent race conditions
    const std::lock_guard<std::recursive_mutex> lock(sim->mtx);

    this->GetState(&this->robot_state);

    this->StateController(&this->robot_state, &this->robot_command);

#ifdef USE_MUJOCO_ROS2
    // Arm mode keys: 2 HOLD, 3 HOME, 4 WBC. GetDown/Passive set damping.
    {
        const auto key = this->control.current_keyboard;
        std::string key_mode;
        if (key == Input::Keyboard::Num2) key_mode = "HOLD";
        else if (key == Input::Keyboard::Num3) key_mode = "HOME";
        else if (key == Input::Keyboard::Num4) key_mode = "WBC";
        else if (key == Input::Keyboard::Num9 || key == Input::Keyboard::P) key_mode = "DAMPING";
        if (!key_mode.empty())
        {
            auto request = std::make_shared<std_msgs::msg::String>();
            request->data = key_mode;
            this->RosArmModeCallback(request);
        }
    }
    // Mirror the real Go2 safety coupling: passive/damping legs imply a
    // damping arm, even if the last operator mode was HOLD or OCS2.
    const bool base_damping = this->fsm.current_state_ &&
        this->fsm.current_state_->GetStateName() == "RLFSMStatePassive";
    if (base_damping)
    {
        std::lock_guard<std::mutex> lock(ros_arm_mutex_);
        if (ros_arm_mode_ != "DAMPING")
        {
            ros_arm_mode_ = "DAMPING";
            this->arm_mode_display = ros_arm_mode_;
            if (ros_arm_mode_pub_)
            {
                std_msgs::msg::String mode;
                mode.data = ros_arm_mode_;
                ros_arm_mode_pub_->publish(mode);
            }
        }
    }
#endif

    if (this->control.current_keyboard == Input::Keyboard::R || this->control.current_gamepad == Input::Gamepad::RB_Y)
    {
        if (this->mj_model && this->mj_data)
        {
            mj_resetData(this->mj_model, this->mj_data);
            mj_forward(this->mj_model, this->mj_data);
        }
    }
    if (this->control.current_keyboard == Input::Keyboard::Enter || this->control.current_gamepad == Input::Gamepad::RB_X)
    {
        if (simulation_running)
        {
            sim->run = 0;
            std::cout << std::endl << LOGGER::INFO << "Simulation Stop" << std::endl;
        }
        else
        {
            sim->run = 1;
            std::cout << std::endl << LOGGER::INFO << "Simulation Start" << std::endl;
        }
        simulation_running = !simulation_running;
    }

#ifdef USE_MUJOCO_ROS2
    {
        std::lock_guard<std::mutex> lock(ros_arm_mutex_);
        const int begin = this->params.Get<int>("num_leg_dofs", 12);
        const int dofs = std::min(6, this->params.Get<int>("num_arm_dofs", 6));
        if (ros_arm_mode_ == "DAMPING") {
            ros_arm_hold_valid_ = false;
            for (int i = 0; i < dofs; ++i) {
                robot_command.motor_command.kp[begin + i] = 0.0f;
                robot_command.motor_command.kd[begin + i] = 0.0f;
                robot_command.motor_command.tau[begin + i] = 0.0f;
            }
        } else if (ros_arm_mode_ == "OCS2" || ros_arm_mode_ == "WBC" || ros_arm_mode_ == "HOLD" || ros_arm_mode_ == "HOME") {
            const auto home = this->params.Get<std::vector<float>>("default_dof_pos");
            // OCS2 with no fresh request yet (just switched in) holds the
            // current pose instead of jumping to a stale/zero target.
            const bool use_target = (ros_arm_mode_ == "OCS2" || ros_arm_mode_ == "WBC") && ros_arm_target_valid_;
            // HOLD latches the pose on entry; re-reading the live pose every
            // tick would give zero position error and let gravity sag the arm.
            if (ros_arm_mode_ != "HOLD") ros_arm_hold_valid_ = false;
            else if (!ros_arm_hold_valid_) {
                for (int i = 0; i < dofs; ++i) ros_arm_hold_q_[i] = robot_state.motor_state.q[begin + i];
                ros_arm_hold_valid_ = true;
            }
            for (int i = 0; i < dofs; ++i) {
                const float q = ros_arm_mode_ == "HOME" ? home[begin + i] :
                    ros_arm_mode_ == "HOLD" ? ros_arm_hold_q_[i] :
                    (use_target ? ros_arm_target_q_[i] : robot_state.motor_state.q[begin + i]);
                robot_command.motor_command.q[begin + i] = q;
                robot_command.motor_command.dq[begin + i] = use_target ? ros_arm_target_dq_[i] : 0.0f;
                robot_command.motor_command.kp[begin + i] = this->params.Get<std::vector<float>>("fixed_kp")[begin + i];
                robot_command.motor_command.kd[begin + i] = this->params.Get<std::vector<float>>("fixed_kd")[begin + i];
            }
        }
    }
    ApplyRosBaseCommand();
#endif
    this->control.ClearInput();
    this->SetCommand(&this->robot_command);
#ifdef USE_MUJOCO_ROS2
    PublishRosArmState();
    PublishRosArmTarget();
    PublishRosOdometry();
#endif
}

#ifdef USE_MUJOCO_ROS2
void RL_Sim::StartRosInterface()
{
    int argc = 0; char **argv = nullptr;
    if (!rclcpp::ok()) rclcpp::init(argc, argv);
    ros_node_ = std::make_shared<rclcpp::Node>("go2_x5_mujoco_ros");
    ros_arm_state_pub_ = ros_node_->create_publisher<sensor_msgs::msg::JointState>("/go2_x5/arm/state", rclcpp::SensorDataQoS());
    ros_arm_mode_pub_ = ros_node_->create_publisher<std_msgs::msg::String>("/go2_x5/arm/mode/state", rclcpp::QoS(1).transient_local());
    ros_arm_command_sub_ = ros_node_->create_subscription<trajectory_msgs::msg::JointTrajectory>(
        "/go2_x5/arm/command/request", 10, std::bind(&RL_Sim::RosArmCommandCallback, this, std::placeholders::_1));
    ros_arm_mode_sub_ = ros_node_->create_subscription<std_msgs::msg::String>(
        "/go2_x5/arm/mode/request", 10, std::bind(&RL_Sim::RosArmModeCallback, this, std::placeholders::_1));
    ros_fsm_key_sub_ = ros_node_->create_subscription<std_msgs::msg::String>(
        "/go2_x5/fsm/key", 10, [this](const std_msgs::msg::String::SharedPtr msg) { this->InjectKey(msg->data); });
    ros_base_command_sub_ = ros_node_->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/go2_x5/base/command", 10, std::bind(&RL_Sim::RosBaseCommandCallback, this, std::placeholders::_1));
    ros_arm_target_pub_ = ros_node_->create_publisher<trajectory_msgs::msg::JointTrajectory>("/go2_x5/arm/command/target", rclcpp::QoS(10));
    ros_odom_pub_ = ros_node_->create_publisher<nav_msgs::msg::Odometry>("/go2_x5/slam/odometry", rclcpp::SensorDataQoS());
    std_msgs::msg::String mode; mode.data = ros_arm_mode_; ros_arm_mode_pub_->publish(mode);
    ros_thread_ = std::thread([this]() { rclcpp::spin(ros_node_); });
}

void RL_Sim::RosArmCommandCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{
    if (msg->points.empty() || msg->points.back().positions.size() < 6) return;
    std::lock_guard<std::mutex> lock(ros_arm_mutex_);
    for (int i = 0; i < 6; ++i) {
        ros_arm_target_q_[i] = static_cast<float>(msg->points.back().positions[i]);
        ros_arm_target_dq_[i] = i < static_cast<int>(msg->points.back().velocities.size()) ? static_cast<float>(msg->points.back().velocities[i]) : 0.0f;
    }
    ros_arm_target_valid_ = true;
}

void RL_Sim::RosArmModeCallback(const std_msgs::msg::String::SharedPtr msg)
{
    std::string mode = msg->data;
    std::transform(mode.begin(), mode.end(), mode.begin(), [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
    if (mode != "HOME" && mode != "HOLD" && mode != "DAMPING" && mode != "OCS2" && mode != "WBC") return;
    {
        std::lock_guard<std::mutex> lock(ros_arm_mutex_);
        const bool was_mpc = ros_arm_mode_ == "OCS2" || ros_arm_mode_ == "WBC";
        if ((mode == "OCS2" || mode == "WBC") && !was_mpc) ros_arm_target_valid_ = false;
        ros_arm_mode_ = mode;
        this->arm_mode_display = mode;
    }
    if (ros_arm_mode_pub_) { std_msgs::msg::String state; state.data = mode; ros_arm_mode_pub_->publish(state); }
}

void RL_Sim::PublishRosArmState()
{
    if (!ros_arm_state_pub_) return;
    const int begin = this->params.Get<int>("num_leg_dofs", 12);
    const int dofs = std::min(6, this->params.Get<int>("num_arm_dofs", 6));
    sensor_msgs::msg::JointState msg; msg.header.stamp = ros_node_->now();
    for (int i = 0; i < dofs; ++i) { msg.name.push_back("x5_joint" + std::to_string(i + 1)); msg.position.push_back(robot_state.motor_state.q[begin + i]); msg.velocity.push_back(robot_state.motor_state.dq[begin + i]); }
    ros_arm_state_pub_->publish(msg);
}

void RL_Sim::RosBaseCommandCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    if (msg->data.size() != 6) return;
    std::lock_guard<std::mutex> lock(ros_arm_mutex_);
    std::copy(msg->data.begin(), msg->data.end(), ros_base_command_.begin());
    ros_base_command_time_ = std::chrono::steady_clock::now();
    ros_base_command_seen_ = true;
}

// WBC mode only: the MPC base channels replace the operator's velocity/pose
// commands, and only while the legs are in the locomotion state. Any other
// mode/state releases them, so leaving WBC always stops the base.
void RL_Sim::ApplyRosBaseCommand()
{
    const bool locomotion = this->fsm.current_state_ &&
        this->fsm.current_state_->GetStateName() == "RLFSMStateRLLocomotion";
    std::array<float, 6> cmd{};
    bool wbc = false, fresh = false;
    {
        std::lock_guard<std::mutex> lock(ros_arm_mutex_);
        wbc = ros_arm_mode_ == "WBC" && ros_arm_target_valid_;
        cmd = ros_base_command_;
        fresh = ros_base_command_seen_ &&
            std::chrono::steady_clock::now() - ros_base_command_time_ < std::chrono::milliseconds(200);
    }
    if (wbc && locomotion)
    {
        if (fresh) this->ApplyExternalBaseCommand(cmd);
        else this->CoastExternalBaseCommand();
        ros_base_driven_ = true;
    }
    else if (ros_base_driven_)
    {
        this->ReleaseExternalBaseCommand();
        ros_base_driven_ = false;
    }
}

// Same message the real-robot RL node sends to the ARX5 driver, so the arm
// target path can be observed identically in simulation.
void RL_Sim::PublishRosArmTarget()
{
    if (!ros_arm_target_pub_) return;
    { std::lock_guard<std::mutex> lock(ros_arm_mutex_); if (ros_arm_mode_ == "DAMPING") return; }
    const int begin = this->params.Get<int>("num_leg_dofs", 12);
    trajectory_msgs::msg::JointTrajectory msg;
    msg.header.stamp = ros_node_->now();
    trajectory_msgs::msg::JointTrajectoryPoint point;
    for (int i = 0; i < 6; ++i) {
        msg.joint_names.push_back("x5_joint" + std::to_string(i + 1));
        point.positions.push_back(robot_command.motor_command.q[begin + i]);
        point.velocities.push_back(robot_command.motor_command.dq[begin + i]);
    }
    point.time_from_start = rclcpp::Duration::from_seconds(std::max(0.01, static_cast<double>(this->params.Get<float>("dt"))));
    msg.points.push_back(point);
    ros_arm_target_pub_->publish(msg);
}

// Canonical /go2_x5/slam/odometry from the MJCF base ground truth (framepos,
// framelinvel, framequat, gyro), standing in for FAST-LIO on hardware.
// Convention matches go2_x5_ocs2_node: world-frame pose, body-frame twist.
void RL_Sim::PublishRosOdometry()
{
    if (!ros_odom_pub_ || !this->params.Get<bool>("use_base_state_sensor", false)) return;
    nav_msgs::msg::Odometry msg;
    msg.header.stamp = ros_node_->now();
    msg.header.frame_id = "odom";
    msg.child_frame_id = "base_link";
    msg.pose.pose.position.x = robot_state.base.position[0];
    msg.pose.pose.position.y = robot_state.base.position[1];
    msg.pose.pose.position.z = robot_state.base.position[2];
    msg.pose.pose.orientation.w = robot_state.imu.quaternion[0];
    msg.pose.pose.orientation.x = robot_state.imu.quaternion[1];
    msg.pose.pose.orientation.y = robot_state.imu.quaternion[2];
    msg.pose.pose.orientation.z = robot_state.imu.quaternion[3];
    msg.twist.twist.linear.x = robot_state.base.lin_vel[0];
    msg.twist.twist.linear.y = robot_state.base.lin_vel[1];
    msg.twist.twist.linear.z = robot_state.base.lin_vel[2];
    msg.twist.twist.angular.x = robot_state.imu.gyroscope[0];
    msg.twist.twist.angular.y = robot_state.imu.gyroscope[1];
    msg.twist.twist.angular.z = robot_state.imu.gyroscope[2];
    ros_odom_pub_->publish(msg);
}
#endif

void RL_Sim::SetupSysJoystick(const std::string& device, int bits)
{
    this->sys_js.reset();
    if (device.empty())
    {
        std::cout << LOGGER::INFO << "Joystick disabled by joystick_device; keyboard control available." << std::endl;
        return;
    }

    this->sys_js = std::make_unique<Joystick>(device);
    if (!this->sys_js->isFound())
    {
        std::cout << LOGGER::WARNING << "Joystick [" << device << "] open failed; keyboard control available." << std::endl;
        this->sys_js.reset();
        return;
    }

    this->sys_js_max_value = (1 << (bits - 1));
    std::cout << LOGGER::INFO << "Joystick listening on [" << device << "]" << std::endl;
}

void RL_Sim::GetSysJoystick()
{
    // Clear all button event states
    for (int i = 0; i < 20; ++i)
    {
        this->sys_js_button[i].on_press = false;
        this->sys_js_button[i].on_release = false;
    }

    // Check if joystick is valid before using
    if (!this->sys_js)
    {
        return;
    }

    while (this->sys_js->sample(&this->sys_js_event))
    {
        if (this->sys_js_event.isButton())
        {
            this->sys_js_button[this->sys_js_event.number].update(this->sys_js_event.value);
        }
        else if (this->sys_js_event.isAxis())
        {
            double normalized = double(this->sys_js_event.value) / this->sys_js_max_value;
            if (std::abs(normalized) < this->axis_deadzone)
            {
                this->sys_js_axis[this->sys_js_event.number] = 0;
            }
            else
            {
                this->sys_js_axis[this->sys_js_event.number] = this->sys_js_event.value;
            }
        }
    }

    if (this->sys_js_button[0].on_press) this->control.SetGamepad(Input::Gamepad::A);
    if (this->sys_js_button[1].on_press) this->control.SetGamepad(Input::Gamepad::B);
    if (this->sys_js_button[2].on_press) this->control.SetGamepad(Input::Gamepad::X);
    if (this->sys_js_button[3].on_press) this->control.SetGamepad(Input::Gamepad::Y);
    if (this->sys_js_button[4].on_press) this->control.SetGamepad(Input::Gamepad::LB);
    if (this->sys_js_button[5].on_press) this->control.SetGamepad(Input::Gamepad::RB);
    if (this->sys_js_button[9].on_press) this->control.SetGamepad(Input::Gamepad::LStick);
    if (this->sys_js_button[10].on_press) this->control.SetGamepad(Input::Gamepad::RStick);
    if (this->sys_js_axis[7] < 0) this->control.SetGamepad(Input::Gamepad::DPadUp);
    if (this->sys_js_axis[7] > 0) this->control.SetGamepad(Input::Gamepad::DPadDown);
    if (this->sys_js_axis[6] > 0) this->control.SetGamepad(Input::Gamepad::DPadLeft);
    if (this->sys_js_axis[6] < 0) this->control.SetGamepad(Input::Gamepad::DPadRight);
    if (this->sys_js_button[4].pressed && this->sys_js_button[0].on_press) this->control.SetGamepad(Input::Gamepad::LB_A);
    if (this->sys_js_button[4].pressed && this->sys_js_button[1].on_press) this->control.SetGamepad(Input::Gamepad::LB_B);
    if (this->sys_js_button[4].pressed && this->sys_js_button[2].on_press) this->control.SetGamepad(Input::Gamepad::LB_X);
    if (this->sys_js_button[4].pressed && this->sys_js_button[3].on_press) this->control.SetGamepad(Input::Gamepad::LB_Y);
    if (this->sys_js_button[4].pressed && this->sys_js_button[9].on_press) this->control.SetGamepad(Input::Gamepad::LB_LStick);
    if (this->sys_js_button[4].pressed && this->sys_js_button[10].on_press) this->control.SetGamepad(Input::Gamepad::LB_RStick);
    if (this->sys_js_button[4].pressed && this->sys_js_axis[7] < 0) this->control.SetGamepad(Input::Gamepad::LB_DPadUp);
    if (this->sys_js_button[4].pressed && this->sys_js_axis[7] > 0) this->control.SetGamepad(Input::Gamepad::LB_DPadDown);
    if (this->sys_js_button[4].pressed && this->sys_js_axis[6] > 0) this->control.SetGamepad(Input::Gamepad::LB_DPadRight);
    if (this->sys_js_button[4].pressed && this->sys_js_axis[6] < 0) this->control.SetGamepad(Input::Gamepad::LB_DPadLeft);
    if (this->sys_js_button[5].pressed && this->sys_js_button[0].on_press) this->control.SetGamepad(Input::Gamepad::RB_A);
    if (this->sys_js_button[5].pressed && this->sys_js_button[1].on_press) this->control.SetGamepad(Input::Gamepad::RB_B);
    if (this->sys_js_button[5].pressed && this->sys_js_button[2].on_press) this->control.SetGamepad(Input::Gamepad::RB_X);
    if (this->sys_js_button[5].pressed && this->sys_js_button[3].on_press) this->control.SetGamepad(Input::Gamepad::RB_Y);
    if (this->sys_js_button[5].pressed && this->sys_js_button[9].on_press) this->control.SetGamepad(Input::Gamepad::RB_LStick);
    if (this->sys_js_button[5].pressed && this->sys_js_button[10].on_press) this->control.SetGamepad(Input::Gamepad::RB_RStick);
    if (this->sys_js_button[5].pressed && this->sys_js_axis[7] < 0) this->control.SetGamepad(Input::Gamepad::RB_DPadUp);
    if (this->sys_js_button[5].pressed && this->sys_js_axis[7] > 0) this->control.SetGamepad(Input::Gamepad::RB_DPadDown);
    if (this->sys_js_button[5].pressed && this->sys_js_axis[6] > 0) this->control.SetGamepad(Input::Gamepad::RB_DPadRight);
    if (this->sys_js_button[5].pressed && this->sys_js_axis[6] < 0) this->control.SetGamepad(Input::Gamepad::RB_DPadLeft);
    if (this->sys_js_button[4].pressed && this->sys_js_button[5].on_press) this->control.SetGamepad(Input::Gamepad::LB_RB);

    auto clamp_to_param = [this](float value, const std::string &key) -> float
    {
        if (!this->params.Has(key)) return value;
        auto limit = this->params.Get<std::vector<float>>(key);
        return std::clamp(value, limit[0], limit[1]);
    };

    float ly = -float(this->sys_js_axis[1]) / float(this->sys_js_max_value);
    float lx = -float(this->sys_js_axis[0]) / float(this->sys_js_max_value);
    float rx = -float(this->sys_js_axis[3]) / float(this->sys_js_max_value);

    bool has_input = (ly != 0.0f || lx != 0.0f || rx != 0.0f);

    if (has_input)
    {
        this->control.x = clamp_to_param(ly, "limit_vel_x");
        this->control.y = clamp_to_param(lx, "limit_vel_y");
        this->control.yaw = clamp_to_param(rx, "limit_vel_yaw");
        this->sys_js_active = true;
    }
    else if (this->sys_js_active)
    {
        this->control.x = 0.0f;
        this->control.y = 0.0f;
        this->control.yaw = 0.0f;
        this->sys_js_active = false;
    }

    // Body pose commands (only observed by policies that ask for them, e.g.
    // RoboDuet's roboduet/dog_commands -- harmless no-op otherwise). Right
    // stick Y sets pitch and the triggers set roll, both proportional and
    // snapping to 0 on release like x/y/yaw above; D-pad up/down ramps height
    // at the same rate as the keyboard's U/J. Axes 2/4/5 (LT/RY/RT) are free
    // -- axis/dpad indices here follow the same js0 layout already used above
    // for LX/LY/RX/DPad. Standard joydev trigger axes rest at -max (released)
    // and read +max at full pull; flip the sign below if a given pad differs.
    float ry = -float(this->sys_js_axis[4]) / float(this->sys_js_max_value);
    float lt = std::clamp((float(this->sys_js_axis[2]) / float(this->sys_js_max_value) + 1.0f) * 0.5f, 0.0f, 1.0f);
    float rt = std::clamp((float(this->sys_js_axis[5]) / float(this->sys_js_max_value) + 1.0f) * 0.5f, 0.0f, 1.0f);

    bool has_pose_input = (ry != 0.0f || lt > 0.01f || rt > 0.01f);

    if (has_pose_input)
    {
        this->control.body_pitch = clamp_to_param(ry, "limit_body_pitch");
        this->control.body_roll = clamp_to_param(0.4f * (rt - lt), "limit_body_roll");
        this->sys_js_pose_active = true;
    }
    else if (this->sys_js_pose_active)
    {
        this->control.body_pitch = 0.0f;
        this->control.body_roll = 0.0f;
        this->sys_js_pose_active = false;
    }

    if (this->sys_js_axis[7] < 0) this->control.body_height = clamp_to_param(this->control.body_height + 0.004f, "limit_body_height");
    if (this->sys_js_axis[7] > 0) this->control.body_height = clamp_to_param(this->control.body_height - 0.004f, "limit_body_height");
}

#ifdef USE_JOYLINK
void RL_Sim::SetupJoyLink(const std::string& config_path)
{
    try
    {
        this->joylink = std::make_unique<joylink_client::JoylinkClient>(config_path);
        if (!this->joylink->connect())
        {
            std::cout << LOGGER::ERROR << "[JoyLink] Failed to connect -- is `joylink " << config_path << "` running?" << std::endl;
        }
    }
    catch (const std::exception& e)
    {
        std::cout << LOGGER::ERROR << "[JoyLink] " << e.what() << std::endl;
        this->joylink.reset();
    }
}

void RL_Sim::GetJoyLinkInput()
{
    if (!this->joylink)
    {
        return;
    }

    // Config (and therefore gait_frequency/stance_width/stance_length) is
    // only loaded once RLFSMStateRLLocomotion::Enter() has run; snapshot the
    // pristine, config.yaml-declared values the first tick after that so the
    // "back" reset below has something to restore to.
    if (!this->joylink_defaults_captured && this->rl_init_done)
    {
        this->joylink_defaults_captured = true;
        this->joylink_default_gait_frequency = this->params.Get<float>("gait_frequency");
        this->joylink_default_stance_width = this->params.Get<float>("stance_width");
        this->joylink_default_stance_length = this->params.Get<float>("stance_length");
    }

    // receive()/receiveRaw() drop backlog and return only the newest sample
    // (fixed upstream in JoyLink -- it used to pop its queue oldest-first
    // with no skip-ahead, so a stalled consumer, e.g. this process's own
    // MuJoCo/torch load hiccuping the 10ms joystick loop, would fall
    // permanently behind, one stale frame per call).
    joystick_common::MappedJoystickData data;
    if (!this->joylink->receive(data, 0))
    {
        return;
    }

    auto axis = [&](const std::string& name) -> float
    {
        auto it = data.axes.find(name);
        return it != data.axes.end() ? it->second : 0.0f;
    };
    auto button_rising = [&](const std::string& name) -> bool
    {
        auto it = data.buttons.find(name);
        int value = (it != data.buttons.end()) ? it->second : 0;
        int prev = this->joylink_prev_buttons.count(name) ? this->joylink_prev_buttons[name] : 0;
        this->joylink_prev_buttons[name] = value;
        return value != 0 && prev == 0;
    };

    // Absolute velocity/pose axes, same roles and scale factors as RoboDuet's
    // play_by_joy.py JOYSTICK_COMMAND_MAP (right_stick_y->yaw, right_stick_x
    // ->pitch, triggers->roll -- deliberately not the raw-joydev path's own
    // right_stick_x->yaw convention, see GetSysJoystick above). Final clamping
    // to the active policy's own limit_vel_x/y/yaw / limit_body_pitch/roll
    // happens generically in RL::StateController(), so only RoboDuet's scale
    // factors are applied here.
    this->control.x = axis("left_stick_x") * 1.5f;
    this->control.y = axis("left_stick_y");
    this->control.yaw = axis("right_stick_y") * 1.5f;
    this->control.body_pitch = axis("right_stick_x") * -1.0f;
    this->control.body_roll = axis("right_trigger") * -0.3f + axis("left_trigger") * 0.3f;

    // D-pad: step-once on threshold crossing. RoboDuet names these
    // "dpad_x"->body_height_delta and "dpad_y"->gait_freq; both are just the
    // hat's two physical axes, kept here under JoyLink's own axis names.
    const float dpad_x = axis("dpad_x");
    const float dpad_y = axis("dpad_y");
    const float kDpadThreshold = 0.5f;
    if (dpad_x > kDpadThreshold && this->joylink_prev_dpad_x <= kDpadThreshold)
    {
        this->control.body_height = clamp(this->control.body_height + 0.05f, -0.3f, 0.3f);
    }
    else if (dpad_x < -kDpadThreshold && this->joylink_prev_dpad_x >= -kDpadThreshold)
    {
        this->control.body_height = clamp(this->control.body_height - 0.05f, -0.3f, 0.3f);
    }
    if (dpad_y > kDpadThreshold && this->joylink_prev_dpad_y <= kDpadThreshold)
    {
        this->params.Set("gait_frequency", YAML::Node(clamp(this->params.Get<float>("gait_frequency") - 0.5f, 1.0f, 8.0f)));
    }
    else if (dpad_y < -kDpadThreshold && this->joylink_prev_dpad_y >= -kDpadThreshold)
    {
        this->params.Set("gait_frequency", YAML::Node(clamp(this->params.Get<float>("gait_frequency") + 0.5f, 1.0f, 8.0f)));
    }
    this->joylink_prev_dpad_x = dpad_x;
    this->joylink_prev_dpad_y = dpad_y;

    // A/B step stance_length, X/Y step stance_width. Both are read straight
    // out of rl.params by rl_sdk.cpp's roboduet/dog_commands term, so a Set()
    // here is immediately what the policy is told next tick -- no separate
    // state to keep in sync, unlike RoboDuet's own fixed-gait play script
    // (whose warning about commands_dog vs. the actual clock doesn't apply
    // here for the same reason gait_frequency doesn't need it either).
    if (button_rising("a"))
    {
        this->params.Set("stance_length", YAML::Node(clamp(this->params.Get<float>("stance_length") - 0.05f, 0.2f, 0.5f)));
    }
    if (button_rising("b"))
    {
        this->params.Set("stance_length", YAML::Node(clamp(this->params.Get<float>("stance_length") + 0.05f, 0.2f, 0.5f)));
    }
    if (button_rising("x"))
    {
        this->params.Set("stance_width", YAML::Node(clamp(this->params.Get<float>("stance_width") - 0.05f, 0.25f, 0.45f)));
    }
    if (button_rising("y"))
    {
        this->params.Set("stance_width", YAML::Node(clamp(this->params.Get<float>("stance_width") + 0.05f, 0.25f, 0.45f)));
    }

    // "back" stands in for play_by_joy.py's f2/reset: that button resets the
    // whole IsaacGym env (physics + commands), which has no equivalent here
    // -- the FSM's own GetDown/GetUp already does the physical reset. This
    // just zeroes velocity/pose and restores the gait shape to what
    // config.yaml declared.
    if (button_rising("back"))
    {
        this->control.x = 0.0f;
        this->control.y = 0.0f;
        this->control.yaw = 0.0f;
        this->control.body_pitch = 0.0f;
        this->control.body_roll = 0.0f;
        this->control.body_height = 0.0f;
        if (this->joylink_defaults_captured)
        {
            this->params.Set("gait_frequency", YAML::Node(this->joylink_default_gait_frequency));
            this->params.Set("stance_width", YAML::Node(this->joylink_default_stance_width));
            this->params.Set("stance_length", YAML::Node(this->joylink_default_stance_length));
        }
        std::cout << std::endl << LOGGER::NOTE << "[JoyLink] Reset commands and gait shape to config defaults" << std::endl;
    }
}
#endif

void RL_Sim::RunModel()
{
    // Keep the state snapshot, response update and inference coherent for the
    // native RC_s17/MRT contract. Other policy paths retain their old timing.
    std::unique_lock<std::recursive_mutex> native_lock(sim->mtx, std::defer_lock);
    if (params.Get<bool>("native_mrt", false)) native_lock.lock();
    if (this->rl_init_done && simulation_running)
    {
        this->episode_length_buf += 1;
        this->obs.ang_vel = this->robot_state.imu.gyroscope;
        this->obs.commands = {this->control.x, this->control.y, this->control.yaw};
        //not currently available for non-ros mujoco version
        // if (this->control.navigation_mode)
        // {
        //     this->obs.commands = {(float)this->cmd_vel.linear.x, (float)this->cmd_vel.linear.y, (float)this->cmd_vel.angular.z};
        // }
        this->obs.base_quat = this->robot_state.imu.quaternion;
        this->obs.dof_pos = this->robot_state.motor_state.q;
        this->obs.dof_vel = this->robot_state.motor_state.dq;
        this->obs.lin_vel = this->robot_state.base.lin_vel;
        this->obs.base_height = {this->robot_state.base.position[2]};

        if (params.Get<bool>("policy_base_at_trunk", false))
        {
            // The v1 transport remains at the IMU origin. RC_s17 and F1 were
            // trained/fitted at the trunk origin, so correct only this actor
            // observation snapshot.
            const float w=obs.base_quat[0], x=obs.base_quat[1], y=obs.base_quat[2], z=obs.base_quat[3];
            const float rx=-.02557f, rz=.04232f;
            obs.base_height[0] -= 2.f*(x*z-w*y)*rx + (1.f-2.f*(x*x+y*y))*rz;
            const auto& omega=obs.ang_vel;
            obs.lin_vel[0] -= omega[1]*rz;
            obs.lin_vel[1] -= omega[2]*rx-omega[0]*rz;
            obs.lin_vel[2] -= -omega[1]*rx;
        }
        if (params.Get<bool>("native_mrt", false) &&
            params.Get<bool>("servo_observation_timing", false))
        {
            AdvanceServoObservation(completed_dog_command_);
        }

        this->obs.actions = this->Forward();
        // Policies may drive fewer joints than the robot has (RoboDuet's dog
        // policy outputs 12 actions for an 18-DoF robot). Zero-pad so every
        // downstream num_of_dofs-wide loop stays in bounds; the padded joints
        // hold their default position via a zero entry in action_scale.
        this->obs.actions.resize(this->params.Get<int>("num_of_dofs"), 0.0f);
        this->ComputeOutput(this->obs.actions, this->output_dof_pos, this->output_dof_vel, this->output_dof_tau);

        if (params.Get<bool>("native_mrt", false))
        {
            completed_dog_command_ = {
                control.x, control.y, control.yaw,
                control.body_pitch, control.body_roll, control.body_height};
            const auto gait = params.Get<std::vector<float>>("dog_commands_extra");
            completed_dog_command_.insert(completed_dog_command_.end(), gait.begin(), gait.end());
        }

        if (!this->output_dof_pos.empty())
        {
            output_dof_pos_queue.push(this->output_dof_pos);
        }
        if (!this->output_dof_vel.empty())
        {
            output_dof_vel_queue.push(this->output_dof_vel);
        }
        if (!this->output_dof_tau.empty())
        {
            output_dof_tau_queue.push(this->output_dof_tau);
        }

        // this->TorqueProtect(this->output_dof_tau);
        // this->AttitudeProtect(this->robot_state.imu.quaternion, 75.0f, 75.0f);

#ifdef CSV_LOGGER
        std::vector<float> tau_est(this->params.Get<int>("num_of_dofs"), 0.0f);
        for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
        {
            tau_est[i] = this->joint_efforts[this->params.Get<std::vector<std::string>>("joint_controller_names")[i]];
        }
        this->CSVLogger(this->output_dof_tau, tau_est, this->obs.dof_pos, this->output_dof_pos, this->obs.dof_vel);
#endif
    }
}

std::vector<float> RL_Sim::Forward()
{
    std::unique_lock<std::mutex> lock(this->model_mutex, std::try_to_lock);

    // If model is being reinitialized, return previous actions to avoid blocking
    if (!lock.owns_lock())
    {
        std::cout << LOGGER::WARNING << "Model is being reinitialized, using previous actions" << std::endl;
        return this->obs.actions;
    }

    std::vector<float> clamped_obs = this->ComputeObservation();

    std::vector<float> actions;
    if (this->params.Get<std::vector<int>>("observations_history").size() != 0)
    {
        this->history_obs_buf.insert(clamped_obs);
        this->history_obs = this->history_obs_buf.get_obs_vec(this->params.Get<std::vector<int>>("observations_history"));
        actions = this->model->forward({this->history_obs});
    }
    else
    {
        actions = this->model->forward({clamped_obs});
    }

    if (!this->params.Get<std::vector<float>>("clip_actions_upper").empty() && !this->params.Get<std::vector<float>>("clip_actions_lower").empty())
    {
        return clamp(actions, this->params.Get<std::vector<float>>("clip_actions_lower"), this->params.Get<std::vector<float>>("clip_actions_upper"));
    }
    else
    {
        return actions;
    }
}

void RL_Sim::Plot()
{
    this->plot_t.erase(this->plot_t.begin());
    this->plot_t.push_back(this->motiontime);
    plt::cla();
    plt::clf();
    for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
    {
        this->plot_real_joint_pos[i].erase(this->plot_real_joint_pos[i].begin());
        this->plot_target_joint_pos[i].erase(this->plot_target_joint_pos[i].begin());
        this->plot_real_joint_pos[i].push_back(mj_data->sensordata[i]);
        // this->plot_target_joint_pos[i].push_back();  // TODO
        plt::subplot(this->params.Get<int>("num_of_dofs"), 1, i + 1);
        plt::named_plot("_real_joint_pos", this->plot_t, this->plot_real_joint_pos[i], "r");
        plt::named_plot("_target_joint_pos", this->plot_t, this->plot_target_joint_pos[i], "b");
        plt::xlim(this->plot_t.front(), this->plot_t.back());
    }
    // plt::legend();
    plt::pause(0.01);
}

// Signal handler for Ctrl+C
void signalHandler(int signum)
{
    std::cout << LOGGER::INFO << "Received signal " << signum << ", exiting..." << std::endl;
    if (RL_Sim::instance && RL_Sim::instance->sim)
    {
        RL_Sim::instance->sim->exitrequest.store(1);
    }
}

int main(int argc, char **argv)
{
    signal(SIGINT, signalHandler);
    RL_Sim rl_sar(argc, argv);
    return 0;
}
