// SPDX-License-Identifier: Apache-2.0
// Dedicated simulation FSM for robot_lab U-Hold (23 measured joints,13 actor actions).
#ifndef FSM_G1_23DOF_UHOLD_HPP
#define FSM_G1_23DOF_UHOLD_HPP
#include "fsm_g1.hpp"

namespace g1_uhold_fsm
{
// A small external pose demonstration, independent of the 13-action actor.
// Each segment starts from the last commanded pose, including mid-motion OFF.
class ArmMotion
{
public:
    void Reset() { enabled = false; value = start = goal = elapsed = 0.; }
    void Toggle()
    {
        enabled = !enabled;
        start = value;
        goal = enabled ? 1. : 0.;
        elapsed = 0.;
    }
    void Advance(double dt)
    {
        const double duration = enabled ? 3. : 2.;
        elapsed = std::min(duration, elapsed + dt);
        const double t = elapsed / duration;
        const double smooth = std::clamp(t*t*t*(10. + t*(-15. + 6.*t)), 0., 1.);
        value = start + (goal-start)*smooth;
        if (enabled && elapsed >= duration)
        {
            start = value;
            goal = 1. - goal;
            elapsed = 0.;
        }
    }
    bool enabled = false;
    double value = 0.;
    // Shoulder pitch/roll/yaw, elbow, wrist roll, then the right arm.
    const std::vector<float> amplitude = {-.25f, .12f, 0.f, .20f, 0.f,
                                        -.25f,-.12f, 0.f, .20f, 0.f};
private:
    double start = 0., goal = 0., elapsed = 0.;
};

class GetUp : public g1_fsm::RLFSMStateGetUp
{
public:
    explicit GetUp(RL* rl) : g1_fsm::RLFSMStateGetUp(rl) {}
    bool pending = true;
    void Enter() override
    {
        rl.rl_init_done = false;
        std::vector<float> stale;
        while (rl.output_dof_pos_queue.try_pop(stale)) {}
        while (rl.output_dof_vel_queue.try_pop(stale)) {}
        while (rl.output_dof_tau_queue.try_pop(stale)) {}
        // Initial FSM Enter runs before StateController binds measured-state pointers.
        pending = true;
        std::cout << "\nU-Hold: A/0 = activate; RB+Y/R = reset.\n";
    }
    void Run() override
    {
        if (pending)
        {
            g1_fsm::RLFSMStateGetUp::Enter();
            pending = false;
        }
        g1_fsm::RLFSMStateGetUp::Run();
    }
    std::string CheckChange() override
    {
        auto next = g1_fsm::RLFSMStateGetUp::CheckChange();
        if (next == state_name_ && percent_getup >= 1.f) return "UHoldPolicy";
        return next == "RLFSMStateRLRoboMimicLocomotion" ? "UHoldPolicy" : next;
    }
};

class Policy : public RLFSMState
{
public:
    explicit Policy(RL* rl) : RLFSMState(*rl, "UHoldPolicy") {}
    bool loaded = false;
    ArmMotion arms;
    int last_episode = 0;
    std::vector<float> default_pose;
    void Enter() override
    {
        loaded = false;
        rl.rl_init_done = false;
        rl.episode_length_buf = 0;
        arms.Reset();
        last_episode = 0;
        rl.config_name = "uhold_s42_20k";
        try
        {
            rl.InitRL(rl.robot_name + "/" + rl.config_name);
            default_pose = rl.params.Get<std::vector<float>>("default_dof_pos");
            loaded = true;
            std::cout << "\nU-Hold seed42 final20k loaded. Left stick = XY; right stick = yaw. "
                      << "T / Y = arm motion (OFF at startup/reset)." << std::endl;
        }
        catch (const std::exception& e)
        {
            std::cerr << "U-Hold load failed: " << e.what() << std::endl;
            rl.fsm.RequestStateChange("RLFSMStateGetUp");
        }
    }
    void Run() override
    {
        if (!loaded) return;
        rl.rl_init_done = true;
        RLControl();
        if (rl.control.current_keyboard == Input::Keyboard::T || rl.control.current_gamepad == Input::Gamepad::Y)
        {
            arms.Toggle();
            // ClearInput otherwise restores last_keyboard and can replay this key.
            if (rl.control.current_keyboard == Input::Keyboard::T)
                rl.control.current_keyboard = rl.control.last_keyboard = Input::Keyboard::None;
            if (rl.control.current_gamepad == Input::Gamepad::Y)
                rl.control.current_gamepad = Input::Gamepad::None;
            std::cout << "\nU-Hold arm motion " << (arms.enabled ? "ON (external pose demo)" : "OFF (return to default)") << std::endl;
        }
        const int episode = rl.episode_length_buf;
        if (rl.simulation_running && episode > last_episode)
            arms.Advance((episode-last_episode)*rl.params.Get<float>("dt")*rl.params.Get<int>("decimation"));
        last_episode = episode;
        for (int i = 0; i < 10; ++i)
        {
            fsm_command->motor_command.q[13+i] = default_pose[13+i] + arms.amplitude[i]*arms.value;
            fsm_command->motor_command.dq[13+i] = 0.f;
        }
        // RLControl consumes positions/velocities; this controller uses motor PD,
        // so the unused torque queue must not grow during interactive playback.
        std::vector<float> unused;
        while (rl.output_dof_tau_queue.try_pop(unused)) {}
    }
    void Exit() override { rl.rl_init_done = false; }
    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X)
            return "RLFSMStatePassive";
        if (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A)
            return "RLFSMStateGetUp";
        if (rl.control.current_keyboard == Input::Keyboard::Num9 || rl.control.current_gamepad == Input::Gamepad::B)
            return "RLFSMStateGetDown";
        return state_name_;
    }
};
}

class G1UHoldFSMFactory : public FSMFactory
{
public:
    explicit G1UHoldFSMFactory(const std::string& initial) : initial_(initial) {}
    std::shared_ptr<FSMState> CreateState(void* context, const std::string& name) override
    {
        RL* rl = static_cast<RL*>(context);
        if (name == "RLFSMStatePassive") return std::make_shared<g1_fsm::RLFSMStatePassive>(rl);
        if (name == "RLFSMStateGetUp") return std::make_shared<g1_uhold_fsm::GetUp>(rl);
        if (name == "RLFSMStateGetDown") return std::make_shared<g1_fsm::RLFSMStateGetDown>(rl);
        if (name == "UHoldPolicy") return std::make_shared<g1_uhold_fsm::Policy>(rl);
        return nullptr;
    }
    std::string GetType() const override { return "g1_23dof_uhold"; }
    std::vector<std::string> GetSupportedStates() const override
    { return {"RLFSMStatePassive", "RLFSMStateGetUp", "RLFSMStateGetDown", "UHoldPolicy"}; }
    std::string GetInitialState() const override { return initial_; }
private:
    std::string initial_;
};
REGISTER_FSM_FACTORY(G1UHoldFSMFactory, "UHoldPolicy")
#endif
