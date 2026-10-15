// SPDX-License-Identifier: Apache-2.0
// Dedicated simulation FSM for robot_lab DUET 23DOF policies (23 measured joints, 13 actor actions).
// Arms are held at the training default pose through zero action scale.
#ifndef FSM_G1_23DOF_DUET_HPP
#define FSM_G1_23DOF_DUET_HPP
#include "fsm_g1.hpp"

namespace g1_duet_fsm
{
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
        std::cout << "\nDUET: A/0 = activate; RB+Y/R = reset.\n";
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
        if (next == state_name_ && percent_getup >= 1.f) return "DuetPolicy";
        return next == "RLFSMStateRLRoboMimicLocomotion" ? "DuetPolicy" : next;
    }
};

class Policy : public RLFSMState
{
public:
    explicit Policy(RL* rl) : RLFSMState(*rl, "DuetPolicy") {}
    bool loaded = false;
    long print_counter = 0;
    float last_posture[5] = {0.f, 0.f, 0.f, 0.f, 0.f};
    void Enter() override
    {
        loaded = false;
        rl.rl_init_done = false;
        rl.episode_length_buf = 0;
        rl.config_name = rl.params.Has("policy_config") ? rl.params.Get<std::string>("policy_config") : "dphase_s43_20k";
        try
        {
            rl.InitRL(rl.robot_name + "/" + rl.config_name);
            loaded = true;
            std::cout << "\nDUET policy " << rl.config_name << " loaded. Left stick = XY; right stick X = yaw rate." << std::endl;
            if (rl.params.Has("gait_posture_nominal"))
                std::cout << "Gait/posture: right stick Y = torso pitch (up = lean forward), RB/LB = roll right/left, "
                             "LT/RT = waist yaw left/right; D-pad up/down = raise/lower torso (walk minimum 0.71 m)." << std::endl;
        }
        catch (const std::exception& e)
        {
            std::cerr << "DUET policy load failed: " << e.what() << std::endl;
            rl.fsm.RequestStateChange("RLFSMStateGetUp");
        }
    }
    void Run() override
    {
        if (!loaded) return;
        rl.rl_init_done = true;
        RLControl();
        if (rl.params.Has("gait_posture_nominal") && ++print_counter % 500 == 0)
        {
            const float roll = rl.control.roll, pitch = rl.control.pitch, waist = rl.control.waist_yaw;
            const float crouch = rl.control.crouch;
            const float height = rl.ComputeObservation()[80];
            if (roll != last_posture[0] || pitch != last_posture[1] || waist != last_posture[2] ||
                crouch != last_posture[3] || height != last_posture[4])
                std::cout << "posture cmd (normalized): roll " << roll << " pitch " << pitch << " waist_yaw " << waist << " crouch " << crouch << " height_m " << height << std::endl;
            last_posture[0] = roll; last_posture[1] = pitch; last_posture[2] = waist; last_posture[3] = crouch; last_posture[4] = height;
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

class G1DuetFSMFactory : public FSMFactory
{
public:
    explicit G1DuetFSMFactory(const std::string& initial) : initial_(initial) {}
    std::shared_ptr<FSMState> CreateState(void* context, const std::string& name) override
    {
        RL* rl = static_cast<RL*>(context);
        if (name == "RLFSMStatePassive") return std::make_shared<g1_fsm::RLFSMStatePassive>(rl);
        if (name == "RLFSMStateGetUp") return std::make_shared<g1_duet_fsm::GetUp>(rl);
        if (name == "RLFSMStateGetDown") return std::make_shared<g1_fsm::RLFSMStateGetDown>(rl);
        if (name == "DuetPolicy") return std::make_shared<g1_duet_fsm::Policy>(rl);
        return nullptr;
    }
    std::string GetType() const override { return "g1_23dof_duet"; }
    std::vector<std::string> GetSupportedStates() const override
    { return {"RLFSMStatePassive", "RLFSMStateGetUp", "RLFSMStateGetDown", "DuetPolicy"}; }
    std::string GetInitialState() const override { return initial_; }
private:
    std::string initial_;
};
REGISTER_FSM_FACTORY(G1DuetFSMFactory, "DuetPolicy")
#endif
