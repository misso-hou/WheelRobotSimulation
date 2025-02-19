#pragma once

#include <iostream>

#include "avoid_base.h"

namespace modules {
namespace control {
namespace algorithm {

using vec2f = Eigen::Vector2f;
using vvec2f = vector<vec2f>;

class AvoidTracking {
 public:
  AvoidTracking(){};
  ~AvoidTracking() {}

 private:
  void UpdateGoalPoint(const int goal_index);

 public:
  void UpdateAgentsVelVecInfo(const int goal_index);
  void SetTrackingPath(const vector<port::CommonPose>& path, const vvec2f& boundary);
  float CalVelocityEnvelope();
  vec2f PoseCtrl(const port::CommonPose& robot_pose, const port::CommonPose& target_pose, const port::Twist& target_cmd);

 private:
  //跟踪数据
  int goal_index_;
  int new_goal_index_;
  vector<port::CommonPose> tracking_path_;
};

}  // namespace algorithm
}  // namespace control
}  // namespace modules