# G1 23DOF 实机入口（尚未在硬件验证）

在 rl_sar 根目录启动：

```sh
./cmake_build/bin/rl_real_g1 <iface> g1_23dof_duet
```

`base_real.yaml` 的 `policy_config` 选择策略，默认 `wpsc_s42_15k`。实机控制周期 5 ms，策略周期 20 ms；关节增益保持训练值。

吊架流程：启动 Passive → 吊架保护下按 A（键盘 0）起身 → 等待 2 秒完成并保持默认姿态 → 降低吊架 → R1+↑（键盘 1）进入策略。LB+X（键盘 P）紧急 Passive，B（键盘 9）GetDown。MuJoCo 虚拟吊架用单向弹性绳连接 torso_link 上背部把手（局部坐标 −0.11, 0, 0.30 m），无旋转弹簧。启动 Passive 时仿真踝关节设为下垂姿态（0.5 rad），稳定后脚底离地约 6.2–6.6 cm。初始锚点高 2.19 m、骨盆高 0.93 m。非策略状态下，手柄 D-pad 下/上降低/升高锚点，约 0.1 m/s；终端键盘 G/H 每次降低/升高 5 mm，连续输入最高约 0.1 m/s。临近落地时建议逐次按 G（约 10 次/秒，平均 0.05 m/s），等脚承重且姿态稳定后再按 1/RB+↑。策略时绳力完全释放；P/0/9 返回非策略状态会在当前锚点重新接绳。R/RB+Y 恢复初始锚点和悬挂高度并重置到 GetUp，仍需明确进入策略。绳长 0.86 m，刚度 6000 N/m，阻尼 600 N·s/m；只用于仿真，硬件需操作真实吊架。

遥控器：左摇杆控制前后/左右，右摇杆 X 控制偏航速度，右摇杆 Y 控制俯仰（假设上推 ry 为正，对应前倾）。R1/L1 控制正/负 roll，L2/R2 数字按键控制正/负腰偏航（以 2.0/s 趋近目标，松开回零），方向键下/上以 1.0/s 增加/减少蹲伏量，范围 [0,1]。进入策略时速度、姿态和蹲伏命令清零。

本入口仅经过构建和 MuJoCo 验证，未连接真实机器人。上硬件前需确认遥控器 ry 符号、23DOF 的 mode_machine 与未映射电机行为，以及 LowState IMU 是否为训练所需 pelvis 源（当前读取 LowState，而非已订阅的 torso IMU）。

### External arm targets and Unitree remote

DDS input is enabled only when `arm_dds_topic` exists. Both DUET bases use
`rt/rl_sar/arm_cmd`, type `unitree_hg::msg::dds_::LowCmd_`. Real uses the existing
interface/domain 0; MuJoCo uses `lo`, domain 1. Ten `motor_cmd[slot].q` values
are absolute radians in slots `[15,16,17,18,19,22,23,24,25,26]`: left shoulder
pitch/roll/yaw, elbow, wrist roll, then right in the same order. Slot 29 `q`
is blend weight clamped to [0,1]. Other message fields are ignored.

Only DuetPolicy applies these targets, clamped to the G1 23DOF MJCF limits.
Weight changes at at most 1/s; final arm position commands change at at most
2 rad/s. After 0.5 s without reception the weight ramps to zero and arms
return to the unchanged training default pose. Arm targets are tracked with training PD gains, so expect approximately 0.2 rad gravity sag (no compensation).
Nonfinite targets/weight are ignored. Activation/deactivation prints once per
weight transition. This does not change the policy's measured arm observations.

Test publisher (50 Hz, duration defaults to 8 seconds):

```sh
cmake_build/bin/g1_arm_dds_pub lo 1 1.0 -0.3 0.18 0 0.5 0 -0.3 -0.18 0 0.5 0 8
```

Usage: `g1_arm_dds_pub <iface> <domain> <weight> <10 targets> [seconds=8] [topic=rt/rl_sar/arm_cmd]`.
Official `rt/arm_sdk` is not used because rl_sar releases Unitree's motion service; official arm examples work by changing their topic constant to `rt/rl_sar/arm_cmd`.
On a real robot use domain 0 with the robot interface.
Its targets and weight must be reviewed before hardware operation.

| Unitree input | Xbox equivalent / function |
| --- | --- |
| Left stick Y / X | vx = ly, vy = -lx |
| Right stick X | yaw rate = -rx |
| Right stick Y | torso pitch = ry |
| L1 / R1 | roll left / right |
| L2 / R2 | waist yaw left / right, symmetric digital ramps at 2/s |
| D-pad up / down | raise / lower torso, normalized full height travel in 1 s |
| A | GetUp |
| B | GetDown |
| L1+X | Passive |
| select | immediate Passive, highest button priority |
| R1+D-pad up | policy after completed GetUp; suppresses roll and height input while held |
| start | policy after completed GetUp, same FSM gate as R1+up |

The existing posture/height printout runs every 500 control ticks when the
command changes. Unitree ry positive-up and the physical stick directions
need hardware confirmation against Xbox. Analog L2 is intentionally unused
because R2 is digital. Hardware DDS delivery, physical tracking, and stability
remain unverified by local simulation.
