#pragma once

#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

// CPU counterpart of RoboDuet response/reference.py and deviation.py.
// All parameters and the FK chain belong to the policy bundle, not the robot defaults.
class ResponseObservation
{
public:
    using Vec3 = std::array<float, 3>;
    using Mat3 = std::array<float, 9>;
    struct Joint { Vec3 xyz, rpy, axis; int index; };
    std::vector<float> xi, rate;

    void Configure(const YAML::Node& cfg, float dt, int num_dofs)
    {
        if (!cfg.IsMap() || cfg["version"].as<int>() != 1)
            throw std::runtime_error("Missing/unsupported response_observation config; re-export this policy");
        dt_ = dt;
        if (!(dt_ > 0)) throw std::runtime_error("response policy dt must be positive");
        names_ = cfg["channels"].as<std::vector<std::string>>();
        omega_ = cfg["omega_n"].as<std::vector<float>>();
        limits_ = cfg["rate_limit"].as<std::vector<float>>();
        scales_ = cfg["obs_scale"].as<std::vector<float>>();
        const auto amplitude = cfg["command_amplitude"].as<std::vector<float>>();
        if (names_.empty() || omega_.size()!=names_.size() || limits_.size()!=names_.size() ||
            scales_.size()!=names_.size() || amplitude.size()!=names_.size())
            throw std::runtime_error("response channel parameter widths disagree");
        const std::vector<std::string> canonical = {"vx", "vy", "wyaw", "height", "pitch"};
        indices_.clear();
        for (size_t i=0; i<names_.size(); ++i)
        {
            auto it=std::find(canonical.begin(), canonical.end(), names_[i]);
            if (it==canonical.end() || std::find(names_.begin(), names_.begin()+i, names_[i])!=names_.begin()+i ||
                !(omega_[i]>0) || !(limits_[i]>0) || !std::isfinite(scales_[i]) || !(amplitude[i]>0))
                throw std::runtime_error("Invalid response channel: " + names_[i]);
            indices_.push_back(static_cast<int>(it-canonical.begin()));
        }
        const auto dev=cfg["deviation"];
        const float tau=dev["tau_s"].as<float>();
        const float warmup=dev["warmup_s"].as<float>();
        deadband_=dev["rate_deadband"].as<float>();
        const float excitation=dev["excitation_fraction"].as<float>();
        if (!(tau>0) || !(warmup>=0) || !(deadband_>=0 && deadband_<1) || !(excitation>=0))
            throw std::runtime_error("Invalid response deviation parameters");
        alpha_=1.0-std::exp(-static_cast<double>(dt_)/tau);
        warmup_=std::max(1, static_cast<int>(std::nearbyint(static_cast<double>(warmup)/dt_)));
        deviation_indices_.clear(); floors_.clear();
        for (const auto& name: dev["channels"].as<std::vector<std::string>>())
        {
            const auto it=std::find(names_.begin(), names_.end(), name);
            if (it==names_.end()) throw std::runtime_error("Unknown response deviation channel: "+name);
            size_t i=it-names_.begin();
            deviation_indices_.push_back(i);
            floors_.push_back(std::pow(amplitude[i]*excitation, 2));
        }
        joints_.clear();
        for (const auto& node: cfg["ee_chain"])
        {
            Joint j{Triple(node["xyz"]), Triple(node["rpy"]), Triple(node["axis"]), node["dof_index"].as<int>()};
            if(j.index < -1 || j.index >= num_dofs) throw std::runtime_error("Invalid EE joint index");
            if(j.index>=0)
            {
                float norm=std::sqrt(j.axis[0]*j.axis[0]+j.axis[1]*j.axis[1]+j.axis[2]*j.axis[2]);
                if (!(norm>0)) throw std::runtime_error("Invalid EE joint axis");
                for(auto& v:j.axis) v/=norm;
            }
            joints_.push_back(j);
        }
        if(joints_.empty()) throw std::runtime_error("Missing EE kinematic chain");
        offset_=Triple(cfg["ee_local_pos"]);
        Reset();
    }

    void Reset()
    {
        xi.assign(names_.size(),0); rate=xi;
        cross_.assign(deviation_indices_.size(),0); square_=cross_; lag_=cross_; lag_steps_=cross_;
        steps_=0; aligned_=false;
    }

    void Step(const std::array<float,5>& command, const std::array<float,5>& measured)
    {
        if(!aligned_)
        {
            for(size_t i=0;i<xi.size();++i) xi[i]=measured[indices_[i]];
            aligned_=true;
        }
        for(size_t i=0;i<xi.size();++i)
        {
            float s=omega_[i]*dt_, decay=std::exp(-s), u=command[indices_[i]], d=xi[i]-u;
            float next_d=decay*(1+s)*d+decay*dt_*rate[i];
            float next_rate=-decay*omega_[i]*omega_[i]*dt_*d+decay*(1-s)*rate[i];
            float clipped=std::clamp(next_rate,-limits_[i],limits_[i]);
            xi[i]=(clipped!=next_rate) ? xi[i]+clipped*dt_ : u+next_d;
            rate[i]=clipped;
        }
        for(size_t j=0;j<deviation_indices_.size();++j)
        {
            size_t i=deviation_indices_[j];
            float y=measured[indices_[i]], u=command[indices_[i]];
            cross_[j]+=alpha_*(y*u-cross_[j]); square_[j]+=alpha_*(u*u-square_[j]);
            if(std::abs(rate[i])>limits_[i]*deadband_)
            {
                float increment=(y-xi[i])*(rate[i]>0 ? 1.f : -1.f);
                lag_[j]+=alpha_*(increment-lag_[j]); lag_steps_[j]+=1;
            }
        }
        ++steps_;
    }

    std::vector<float> State() const { return Scaled(xi); }
    std::vector<float> Rate() const
    {
        auto result=rate;
        for(size_t i=0;i<result.size();++i) result[i]/=limits_[i];
        return result;
    }
    std::vector<float> Error(const std::array<float,5>& command) const
    {
        auto result=xi;
        for(size_t i=0;i<result.size();++i) result[i]-=command[indices_[i]];
        return Scaled(result);
    }
    std::vector<float> Deviation() const
    {
        std::vector<float> result(2*deviation_indices_.size(),0);
        if(steps_<warmup_) return result;
        for(size_t j=0;j<deviation_indices_.size();++j)
        {
            if(square_[j]>floors_[j]) result[2*j]=cross_[j]/(square_[j]+1e-4f)-1;
            if(lag_steps_[j]>=1) result[2*j+1]=lag_[j];
        }
        return result;
    }
    std::vector<float> EndEffector(const std::vector<float>& q) const
    {
        Mat3 rot={1,0,0,0,1,0,0,0,1}; Vec3 pos={0,0,0};
        for(const auto& j:joints_)
        {
            auto delta=Apply(rot,j.xyz);
            for(int k=0;k<3;++k) pos[k]+=delta[k];
            rot=Multiply(rot,Multiply(Rotation({0,0,1},j.rpy[2]),
                         Multiply(Rotation({0,1,0},j.rpy[1]),Rotation({1,0,0},j.rpy[0]))));
            if(j.index>=0) rot=Multiply(rot,Rotation(j.axis,q.at(j.index)));
        }
        auto delta=Apply(rot,offset_);
        return {pos[0]+delta[0],pos[1]+delta[1],pos[2]+delta[2]};
    }
private:
    static Vec3 Triple(const YAML::Node& n)
    {
        auto v=n.as<std::vector<float>>();
        if(v.size()!=3 || !std::all_of(v.begin(),v.end(),[](float x){return std::isfinite(x);}))
            throw std::runtime_error("EE transform requires three finite values");
        return {v[0],v[1],v[2]};
    }
    static Mat3 Rotation(Vec3 a,float t)
    {
        float c=std::cos(t),s=std::sin(t),v=1-c,x=a[0],y=a[1],z=a[2];
        return {c+x*x*v,x*y*v-z*s,x*z*v+y*s,y*x*v+z*s,c+y*y*v,y*z*v-x*s,z*x*v-y*s,z*y*v+x*s,c+z*z*v};
    }
    static Mat3 Multiply(const Mat3& a,const Mat3& b)
    {
        Mat3 r{};
        for(int i=0;i<3;++i) for(int j=0;j<3;++j) for(int k=0;k<3;++k) r[3*i+j]+=a[3*i+k]*b[3*k+j];
        return r;
    }
    static Vec3 Apply(const Mat3& a,const Vec3& b)
    {
        Vec3 r{}; for(int i=0;i<3;++i) for(int k=0;k<3;++k) r[i]+=a[3*i+k]*b[k]; return r;
    }
    std::vector<float> Scaled(std::vector<float> v) const
    {
        for(size_t i=0;i<v.size();++i) v[i]*=scales_[i]; return v;
    }
    std::vector<std::string> names_;
    std::vector<int> indices_;
    std::vector<size_t> deviation_indices_;
    std::vector<float> omega_,limits_,scales_,floors_,cross_,square_,lag_,lag_steps_;
    std::vector<Joint> joints_;
    Vec3 offset_{};
    float dt_=0,alpha_=0,deadband_=0;
    int warmup_=1,steps_=0;
    bool aligned_=false;
};
