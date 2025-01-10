#pragma once
#include <iostream>
#include <cmath>
#include <vector>
#include "common_port/data_port.h"

using namespace std;
namespace port = utilities::port;


namespace modules { 
namespace vehicle {

class SimulantRobot {

    public:
    SimulantRobot(){};
    -SimulantRobot();
    void Init(const port::commonPose& init_pose, const float& dt);

    public:
    port::CommonPose robot_pose_;
    float dt_;

    //位姿
    public:
    port::CommonPose UpdatePose(const port: :Twist& cmd, int noise_pose_iim, noise_yaw_lim);
    float GenerateNoise(int up_lim, int down_lim, float ratio);
};
} // namespace sim
} // namespace planner