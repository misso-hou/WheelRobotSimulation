#pragma once

#include <cmath>
#include <iostream>
#include <vector>

#include "common_port/data_port.h"

namespace modules {
namespace vehicle {

using namespace std;
namespace port = utilities::port;

class SimulantRobot {
 public:
  SimulantRobot(){};
  ~SimulantRobot(){};
  void Init(const port::CommonPose& init_pose, const float& dt);

 public:
  port::CommonPose robot_pose_;
  float dt_;

  //位姿
 public:
  port::CommonPose UpdatePose(const port::Twist& cmd, int noise_pose_lim, int noise_yaw_lim);
  float GenerateNoise(int up_lim, int down_lim, float ratio);
};
}  // namespace vehicle
}  // namespace modules