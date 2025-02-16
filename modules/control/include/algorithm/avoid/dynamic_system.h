#pragma once

#include <iostream>

#include "avoid_base.h"

namespace modules {
namespace control {
namespace algorithm {

using namespace std;
namespace port = utilities::port;
//变量名简化
using vec2f = Eigen::Vector2f;
using mat2f = Eigen::Matrix2f;
using vvec2f = vector<vec2f>;
using vvcp = vector<vector<port::ClusterPoint>>;

// SEE NOTES:增加->朝向障碍物提前转向，反之相反，过大会导致tail effect出问题，障碍物吸住机器
#define REACT_FACTOR 0.2f  //流场响应系数
#define SAFE_FACTOR 7      //突破安全边界保护系数
#define CRITICAL_DIS 1.0f  //多障碍物决断距离
#define WEIGHT_POWER 2.0f  //多障碍物影响因子

class DynamicSys {
 public:
  DynamicSys(const float& rf, const int sf, const float& cd, const float& wp);
  ~DynamicSys(){};

 public:
  port::Twist DynamicSysAvoid(const float& planning_v);

 private:
  port::AvoidDir CalAvoidDir(const port::VirtualObs& vt_obs);
  vec2f LimitIntersectionAngle(const vec2f& benchmark_vec, const vec2f& object_vec, const float& angle_thd);
  void CalAgentsCalObsBasicInfo(port::CircleAgent& agent);
  void ComputeAgentsCalObsInfo();
  void CalObsWeights(vector<port::CalculatedObs>& obstacles);
  void CalStrechingMat(port::CircleAgent& agent);
  void CalAgentModulatedV(port::CircleAgent& agent);
  port::Twist SynthesizeAvoidCmd(const float& planning_v);

 private:
  float react_factor_;  //流场响应系数
  int safe_factor_;     //突破安全边界保护系数
  float critical_dis_;  //多障碍物决断距离
  float weight_power_;  //多障碍物影响因子
};

}  // namespace algorithm
}  // namespace control
}  // namespace modules