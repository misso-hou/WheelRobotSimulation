#pragma once

#include <iostream>

#include "obstacle.h"

using namespace std;
namespace port = utilities::port;

namespace modules {
namespace env {

class Environment {
 public:
  Environment(){};
  ~Environment(){};

 public:
  void InitEnv(int obs_num, const port::CommonPose& pose);
  void EnvUpdate();  //障碍物位姿更新
  vector<pair<vector<float>, vector<float>>> GetShowObs();
  vector<Eigen::Vector2f> GetObsPoints();

 private:
  vector<Obstacle> obs_agents_;
  vector<pair<vector<float>, vector<float>>> show_obs_;
  vector<Eigen::Vector2f> obs_points_cloud_;
};

}  // namespace env
}  // namespace modules
