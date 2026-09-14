// Bounded MuJoCo physics evaluation using the production RL observation/model/output code.
#include "rl_sdk.hpp"
#include <mujoco/mujoco.h>
#include <array>
#include <cmath>
#include <deque>
#include <filesystem>
#include <iomanip>
#include <random>

namespace fs = std::filesystem;

class Evaluation : public RL
{
public:
    mjModel* m = nullptr;
    mjData* d = nullptr;
    int body = -1;
    std::vector<int> joints, actuators;
    std::vector<float> raw_obs;
    std::vector<float> raw_actions;
    double ground = 0.0;
    bool imu_frame = false;

    ~Evaluation() { if (d) mj_deleteData(d); if (m) mj_deleteModel(m); }

    void Load(const std::string& key, const std::string& scene)
    {
        robot_name = "go2_x5";
        ReadYaml(robot_name, "base.yaml");
        InitRL(key);
        char error[2048] = {};
        m = mj_loadXML(scene.c_str(), nullptr, error, sizeof(error));
        if (!m) throw std::runtime_error(error);
        m->opt.timestep = 0.0025;
        d = mj_makeData(m);
        body = mj_name2id(m, mjOBJ_BODY, "base_link");
        if (body < 0 || m->nu != params.Get<int>("num_of_dofs"))
            throw std::runtime_error("MJCF does not match the policy robot");
        const auto map = params.Get<std::vector<int>>("joint_mapping");
        for (int i = 0; i < m->nu; ++i)
        {
            const int actuator = map.at(i);
            const int joint = m->actuator_trnid[2 * actuator];
            joints.push_back(joint);
            actuators.push_back(actuator);
        }
        const double control_dt = params.Get<float>("dt");
        if (std::abs(control_dt / m->opt.timestep - std::round(control_dt / m->opt.timestep)) > 1e-5)
            throw std::runtime_error("Control period must be an integer number of physics steps");
    }

    double GroundHeight(double x, double y) const
    {
        const int floor = mj_name2id(m, mjOBJ_GEOM, "floor");
        if (floor < 0 || m->geom_type[floor] != mjGEOM_HFIELD) return 0.0;
        const int h = m->geom_dataid[floor];
        const double* size = m->hfield_size + 4 * h;
        const int nx = m->hfield_ncol[h], ny = m->hfield_nrow[h];
        double u = std::clamp((x - m->geom_pos[3*floor] + size[0]) / (2 * size[0]) * (nx-1), 0.0, double(nx-1)-1e-8);
        double v = std::clamp((y - m->geom_pos[3*floor+1] + size[1]) / (2 * size[1]) * (ny-1), 0.0, double(ny-1)-1e-8);
        const int ix = int(u), iy = int(v);
        const float* z = m->hfield_data + m->hfield_adr[h];
        const double tx = u-ix, ty = v-iy;
        // MuJoCo hfield cell triangles share the lower-left / upper-right diagonal.
        const double z00=z[iy*nx+ix], z10=z[iy*nx+ix+1], z01=z[(iy+1)*nx+ix], z11=z[(iy+1)*nx+ix+1];
        const double z_interp = tx >= ty ? z00+(z10-z00)*tx+(z11-z10)*ty
                                             : z00+(z11-z01)*tx+(z01-z00)*ty;
        return m->geom_pos[3*floor+2] + z_interp*size[2];
    }

    void GetState(RobotState<float>* state) override
    {
        state->motor_state.resize(m->nu);
        for (int i=0; i<m->nu; ++i)
        {
            state->motor_state.q[i] = d->qpos[m->jnt_qposadr[joints[i]]];
            state->motor_state.dq[i] = d->qvel[m->jnt_dofadr[joints[i]]];
            state->motor_state.tau_est[i] = d->actuator_force[actuators[i]];
        }
        const int site = mj_name2id(m, mjOBJ_SITE, "imu");
        mjtNum velocity[6];
        // Body frame at the training base origin. The GUI's IMU-site mode is
        // available as an explicit diagnostic, not silently mixed into scores.
        mj_objectVelocity(m, d, imu_frame ? mjOBJ_SITE : mjOBJ_XBODY,
                          imu_frame ? site : body, velocity, 1);
        for (int i=0; i<4; ++i) state->imu.quaternion[i] = d->xquat[4*body+i];
        for (int i=0; i<3; ++i)
        {
            state->imu.gyroscope[i] = velocity[i];
            state->base.lin_vel[i] = velocity[i+3];
            state->base.position[i] = imu_frame ? d->site_xpos[3*site+i] : d->xpos[3*body+i];
        }
        ground = 0.0;
        const auto euler = QuaternionToEuler(state->imu.quaternion);
        // Same 17 x 11 yaw-rotated height sampling footprint as the snapshot.
        for(int ix=-8;ix<=8;++ix) for(int iy=-5;iy<=5;++iy)
        {
            const double x=0.1*ix,y=0.1*iy;
            ground += GroundHeight(d->xpos[3*body]+std::cos(euler[2])*x-std::sin(euler[2])*y,
                                   d->xpos[3*body+1]+std::sin(euler[2])*x+std::cos(euler[2])*y)/187.0;
        }
    }

    void SetCommand(const RobotCommand<float>* cmd) override
    {
        for (int i=0; i<m->nu; ++i)
        {
            const int j=joints[i];
            const double torque = cmd->motor_command.tau[i]
                + cmd->motor_command.kp[i]*(cmd->motor_command.q[i]-d->qpos[m->jnt_qposadr[j]])
                + cmd->motor_command.kd[i]*(cmd->motor_command.dq[i]-d->qvel[m->jnt_dofadr[j]]);
            const float limit=params.Get<std::vector<float>>("torque_limits")[i];
            d->ctrl[actuators[i]] = std::clamp(torque, -double(limit), double(limit));
        }
    }

    std::vector<float> Forward() override
    {
        raw_obs = ComputeObservation();
        history_obs_buf.insert(raw_obs);
        history_obs = history_obs_buf.get_obs_vec(params.Get<std::vector<int>>("observations_history"));
        raw_actions = model->forward({history_obs});
        if (raw_actions.size() != 12) throw std::runtime_error("Actor action width mismatch");
        for(float value: raw_actions) if(!std::isfinite(value)) throw std::runtime_error("Nonfinite policy action");
        return clamp(raw_actions, params.Get<std::vector<float>>("clip_actions_lower"),
                                  params.Get<std::vector<float>>("clip_actions_upper"));
    }

    void Reset(int seed, const YAML::Node& spec)
    {
        mj_resetData(m,d);
        auto q=params.Get<std::vector<float>>("default_dof_pos");
        for(int i=0;i<m->nu;++i) d->qpos[m->jnt_qposadr[joints[i]]]=q[i];
        std::mt19937 rng(seed);
        std::uniform_real_distribution<double> small(-0.02,0.02);
        d->qpos[2]=0.34+GroundHeight(0,0);
        const double roll=small(rng), pitch=small(rng);
        d->qpos[3]=std::cos(roll/2)*std::cos(pitch/2);
        d->qpos[4]=std::sin(roll/2)*std::cos(pitch/2);
        d->qpos[5]=std::cos(roll/2)*std::sin(pitch/2);
        d->qpos[6]=-std::sin(roll/2)*std::sin(pitch/2);
        mj_forward(m,d);
        InitControl();
        InitObservations();
        gait_indices=0;
        episode_length_buf=0;
        history_obs_buf=ObservationBuffer(1,obs_dims,30,"time");
        ClearExternalArmTarget();
        InitOutputs();
        // Existing comparison suites use common gait values. Deployment checks
        // can retain the loaded policy bundle, including its swing height.
        if (!spec["use_policy_gait_commands"].as<bool>(false))
        {
            params.Set("gait_frequency",YAML::Node(2.0));
            params.Set("footswing_height",YAML::Node(0.04));
            params.Set("stance_width",YAML::Node(0.275));
            params.Set("stance_length",YAML::Node(0.35));
            params.Set("gait_duration",YAML::Node(0.495));
        }
        auto kp=params.Get<std::vector<float>>("fixed_kp");
        auto kd=params.Get<std::vector<float>>("fixed_kd");
        robot_command.motor_command.q=q;
        robot_command.motor_command.kp=kp;
        robot_command.motor_command.kd=kd;
        // A repeatable 2 s PD stand before switching to the policy.
        for(int step=0;step<800;++step) { SetCommand(&robot_command); mj_step(m,d); }
        GetState(&robot_state);
        if(d->xpos[3*body+2]-ground<0.17) throw std::runtime_error("PD initialization failed");
    }
};

int main(int argc,char** argv)
{
    if(argc!=5) {std::cerr<<"Usage: rl_mujoco_eval policy_key scene.xml evaluation.yaml output_directory\n";return 2;}
    try
    {
        Evaluation env;
        env.Load(argv[1],argv[2]);
        auto spec=YAML::LoadFile(argv[3]);
        fs::path output=argv[4];fs::create_directories(output);
        YAML::Node summaries;
        const auto phases=spec["commands"];
        double duration=0;for(auto phase:phases) duration+=phase["seconds"].as<double>();
        const double dt=env.params.Get<float>("dt")*env.params.Get<int>("decimation");
        const int physics_per_policy=std::lround(dt/env.m->opt.timestep);
        const int physics_per_control=std::lround(env.params.Get<float>("dt")/env.m->opt.timestep);
        std::vector<double> original_mass(env.m->body_mass,env.m->body_mass+env.m->nbody);
        std::vector<double> original_inertia(env.m->body_inertia,env.m->body_inertia+env.m->nbody*3);
        for(auto scenario:spec["scenarios"]) for(auto seed_node:spec["seeds"])
        {
            const int seed=seed_node.as<int>();
            const std::string name=scenario["name"].as<std::string>();
            const int latency=scenario["latency_steps"].as<int>(0);
            const bool arm=scenario["arm_motion"].as<bool>(false);
            const bool pushes=scenario["pushes"].as<bool>(false);
            env.imu_frame=scenario["imu_frame"].as<bool>(false);
            const double mass_scale=scenario["arm_mass_scale"].as<double>(1.0);
            for(int b=1;b<env.m->nbody;++b)
            {
                const char* body_name=mj_id2name(env.m,mjOBJ_BODY,b);
                const bool is_arm=body_name && std::string(body_name).find("x5_")==0;
                env.m->body_mass[b]=original_mass[b]*(is_arm?mass_scale:1.0);
                for(int j=0;j<3;++j) env.m->body_inertia[3*b+j]=original_inertia[3*b+j]*(is_arm?mass_scale:1.0);
            }
            mj_setConst(env.m,env.d);
            env.Reset(seed,spec);
            std::deque<RobotState<float>> sensor_history(latency+1,env.robot_state);
            std::deque<double> ground_history(latency+1,env.ground);
            auto home=env.params.Get<std::vector<float>>("default_dof_pos");
            auto kp=env.params.Get<std::vector<float>>("rl_kp");
            auto kd=env.params.Get<std::vector<float>>("rl_kd");
            env.robot_command.motor_command.kp=kp;env.robot_command.motor_command.kd=kd;
            std::ofstream csv(output/(name+"_seed"+std::to_string(seed)+".csv"));
            csv<<std::setprecision(9)<<"time,phase,phase_age,vx_cmd,vy_cmd,yaw_cmd,pitch_cmd,roll_cmd,height_cmd,vx,vy,yaw_rate,pitch,roll,height,x,y,power_w,push,failed";
            for(int j=0;j<env.m->nq;++j)csv<<",qpos"<<j;
            csv<<'\n';
            std::ofstream probe;
            if(spec["probe"].as<bool>(false))
            {
                probe.open(output/(name+"_seed"+std::to_string(seed)+"_probe.csv"));
                probe<<std::setprecision(9);
            }
            int phase_id=0;double phase_start=0,phase_end=phases[0]["seconds"].as<double>();
            double failed_duration=0;
            bool failed=false;int steps=0,push_count=0;
            const int total_steps=std::lround(duration/dt);
            std::mt19937 rng(seed+1000);
            std::uniform_real_distribution<double> sign(-1,1);
            for(int step=0;step<total_steps;++step)
            {
                const double t=step*dt;
                while(t+1e-6>=phase_end && phase_id+1<int(phases.size()))
                {phase_start=phase_end;++phase_id;phase_end+=phases[phase_id]["seconds"].as<double>();}
                auto command=phases[phase_id]["values"].as<std::vector<float>>();
                env.control.x=command[0];env.control.y=command[1];env.control.yaw=command[2];
                env.control.body_pitch=command[3];env.control.body_roll=command[4];env.control.body_height=command[5];
                bool pushed=false;
                if(pushes && step>=500 && step%500==0)
                {
                    // Reproducible world-frame velocity increment, not a force.
                    env.d->qvel[0]+=0.5*sign(rng);env.d->qvel[1]+=0.5*sign(rng);
                    env.d->qvel[3]+=0.4*sign(rng);env.d->qvel[4]+=0.4*sign(rng);env.d->qvel[5]+=0.4*sign(rng);
                    mj_forward(env.m,env.d);pushed=true;++push_count;
                }
                env.GetState(&env.robot_state);
                sensor_history.push_back(env.robot_state);sensor_history.pop_front();
                ground_history.push_back(env.ground);ground_history.pop_front();
                const auto& sensed=sensor_history.front();
                env.obs.base_quat=sensed.imu.quaternion;env.obs.ang_vel=sensed.imu.gyroscope;
                env.obs.lin_vel=sensed.base.lin_vel;env.obs.dof_pos=sensed.motor_state.q;env.obs.dof_vel=sensed.motor_state.dq;
                env.obs.base_height={float(sensed.base.position[2]-ground_history.front())};
                env.obs.commands={command[0],command[1],command[2]};
                const auto sensed_euler=QuaternionToEuler(env.obs.base_quat);
                env.obs.body_pose_actual={env.obs.base_height[0],sensed_euler[1],sensed_euler[0]};
                if(arm)
                {
                    std::vector<float> q(6),dq(6,0);
                    const double ramp=std::min(1.0,t/2.0);
                    const double amplitudes[6]={0.5,0.25,0.25,0.35,0.3,0.35};
                    for(int i=0;i<6;++i)
                    {
                        q[i]=home[12+i]+ramp*amplitudes[i]*std::sin(2*M_PI*(0.18+0.025*i)*t+seed*0.7+i);
                        const int j=env.joints[12+i];
                        q[i]=std::clamp(q[i],float(env.m->jnt_range[2*j]+0.03),float(env.m->jnt_range[2*j+1]-0.03));
                    }
                    env.SetExternalArmTarget(q,dq);
                }
                ++env.episode_length_buf;
                const auto previous_actions=env.obs.actions;
                env.obs.actions=env.Forward();env.obs.actions.resize(env.m->nu,0);
                env.ComputeOutput(env.obs.actions,env.output_dof_pos,env.output_dof_vel,env.output_dof_tau);
                env.robot_command.motor_command.q=env.output_dof_pos;
                env.robot_command.motor_command.dq=env.output_dof_vel;
                if(probe.is_open() && step<1000)
                {
                    auto write=[&](const auto& values){for(auto value:values)probe<<value<<',';};
                    probe<<step<<',';write(command);write(env.obs.base_quat);write(env.obs.ang_vel);write(env.obs.lin_vel);
                    probe<<env.obs.base_height[0]<<',';write(env.obs.dof_pos);write(env.obs.dof_vel);write(previous_actions);
                    write(env.raw_obs);write(env.raw_actions);probe<<env.gait_indices<<'\n';
                }
                double power=0;
                for(int sub=0;sub<physics_per_policy;++sub)
                {
                    if(sub%physics_per_control==0)env.SetCommand(&env.robot_command);
                    mj_step(env.m,env.d);
                    for(int j=0;j<12;++j)power+=std::abs(env.d->actuator_force[env.actuators[j]]*env.d->qvel[env.m->jnt_dofadr[env.joints[j]]])/physics_per_policy;
                }
                // Refresh positions and velocities at the same post-step time.
                mj_forward(env.m,env.d);env.GetState(&env.robot_state);
                auto euler=QuaternionToEuler(env.robot_state.imu.quaternion);
                const double height=env.d->xpos[3*env.body+2]-env.ground;
                const bool unsafe=height<0.17 || std::abs(euler[0])>M_PI/3 || std::abs(euler[1])>M_PI/3;
                failed_duration=unsafe?failed_duration+dt:0;
                failed=failed_duration>=0.2;
                for(int j=0;j<env.m->nq;++j)if(!std::isfinite(env.d->qpos[j]))failed=true;
                csv<<t+dt<<','<<phase_id<<','<<t-phase_start;
                for(auto value:command)csv<<','<<value;
                csv<<','<<env.robot_state.base.lin_vel[0]<<','<<env.robot_state.base.lin_vel[1]<<','<<env.robot_state.imu.gyroscope[2]
                   <<','<<euler[1]<<','<<euler[0]<<','<<height<<','<<env.d->xpos[3*env.body]<<','<<env.d->xpos[3*env.body+1]
                   <<','<<power<<','<<pushed<<','<<failed;
                for(int j=0;j<env.m->nq;++j)csv<<','<<env.d->qpos[j];csv<<'\n';
                ++steps;if(failed)break;
            }
            YAML::Node summary;
            summary["scenario"]=name;summary["seed"]=seed;summary["policy"]=argv[1];
            summary["completed"]=!failed;summary["duration_s"]=steps*dt;summary["requested_duration_s"]=duration;
            summary["push_count"]=push_count;summary["latency_steps"]=latency;summary["arm_motion"]=arm;
            summary["arm_mass_scale"]=mass_scale;summary["state_frame"]=env.imu_frame?"imu":"base_origin";
            summary["physics_dt"]=env.m->opt.timestep;summary["policy_dt"]=dt;
            summary["mujoco_version"]=mj_versionString();summary["scene"]=argv[2];
            summaries.push_back(summary);
            std::ofstream(output/"summary.yaml")<<summaries;
            std::cout<<name<<" seed="<<seed<<" time="<<steps*dt<<" completed="<<!failed<<std::endl;
        }
    }
    catch(const std::exception& error){std::cerr<<error.what()<<std::endl;return 1;}
}
