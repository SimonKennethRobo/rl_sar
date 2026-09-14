// Exercise the production SDK at stand/walk boundaries without a simulator.
#include "rl_sdk.hpp"
#include <iomanip>
#include <fstream>

class GaitProbe : public RL
{
public:
    std::vector<float> Forward() override { return {}; }
    void GetState(RobotState<float>*) override {}
    void SetCommand(const RobotCommand<float>*) override {}
};

int main(int argc, char** argv)
{
    if (argc != 3) return 2;
    GaitProbe rl;
    rl.ReadYaml("go2_x5", "base.yaml");
    rl.InitRL(argv[1]);
    rl.params.Set("observations", YAML::Load("[roboduet/dog_commands, roboduet/clock_inputs]"));
    rl.params.Set("num_observations", YAML::Node(15));
    std::ofstream out(argv[2]);
    out << std::setprecision(9);
    int dynamic;
    float phase, frequency, duration;
    while (std::cin >> dynamic >> phase >> frequency >> duration >> rl.control.x >> rl.control.y >> rl.control.yaw)
    {
        // Negative phase keeps the previous phase to check restart continuity.
        if (phase >= 0) rl.gait_indices = phase;
        rl.params.Set("use_dynamic_gait", dynamic < 0 ? YAML::Node(YAML::NodeType::Undefined) : YAML::Node(bool(dynamic)));
        rl.params.Set("gait_frequency", YAML::Node(frequency));
        rl.params.Set("gait_duration", YAML::Node(duration));
        const auto observation = rl.ComputeObservation();
        out << rl.gait_indices << ' ' << rl.params.Get<float>("gait_frequency");
        for (float value : observation) out << ' ' << value;
        out << '\n';
    }
}
