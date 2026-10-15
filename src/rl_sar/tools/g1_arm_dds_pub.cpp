// SPDX-License-Identifier: Apache-2.0
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <chrono>
#include <thread>
#include <iostream>
int main(int argc, char** argv)
{
    if (argc < 14 || argc > 16) {
        std::cerr << "Usage: g1_arm_dds_pub <iface> <domain> <weight> <10 targets> [seconds=8] [topic=rt/rl_sar/arm_cmd]\n";
        return 1;
    }
    unitree::robot::ChannelFactory::Instance()->Init(std::stoi(argv[2]), argv[1]);
    unitree::robot::ChannelPublisher<unitree_hg::msg::dds_::LowCmd_> pub(argc == 16 ? argv[15] : "rt/rl_sar/arm_cmd");
    pub.InitChannel();
    unitree_hg::msg::dds_::LowCmd_ msg;
    const int slots[] = {15,16,17,18,19,22,23,24,25,26};
    msg.motor_cmd()[29].q() = std::stof(argv[3]);
    for (int i=0; i<10; ++i) msg.motor_cmd()[slots[i]].q() = std::stof(argv[4+i]);
    const double seconds = argc >= 15 ? std::stod(argv[14]) : 8.;
    const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
    while (std::chrono::steady_clock::now() < end) {
        pub.Write(msg);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}
