#pragma once

#include <thread>

#include "algorithm/avoid/avoid_base.h"
#include "algorithm/avoid/avoid_tracking.h"
#include "algorithm/avoid/deal_with_obs.h"
#include "algorithm/avoid/dynamic_system.h"

namespace modules {
namespace control {
namespace function {

using vec2f = Eigen::Vector2f;

class AvoidCenter {
 public:
  AvoidCenter();
  ~AvoidCenter(){};

 private:
  void InitAvoidbase();
  void UpdateAvoidBaseData();
  void SetInterfaceInfo();

 public:
  void ShutdownAvoid();
  void SetTrackingPath(const vector<port::CommonPose>& path);
  void SetRealTimeInfo(const vec2f& ref_vel, const int goal_id);
  int32_t AvoidThreadFun();
  port::Twist GetAvoidCmd();
  int GetAvoidGoalId();
  port::Twist PoseControl(const port::CommonPose& robot_pose, const port::CommonPose target_pose, const port::Twist target_cmd);

 private:
  shared_ptr<algorithm::DynamicSys> dyn_sys_ptr_;  //动态系统算法
  shared_ptr<algorithm::AvoidTracking> avt_ptr_;   //避障跟踪
  shared_ptr<algorithm::DealWithObs> dw_obs_ptr_;  //障碍物处理

 private:
  port::CommonPose interface_pose_;
  vec2f interface_cur_cmd_;
  vec2f interface_ref_cmd_;
  int interface_goal_id_;
  port::Twist avoid_cmd_;
  int goal_id_;

 private:
  std::thread avoid_thread_;
  std::mutex thread_lock_;
  bool avoid_enable_;
};

}  // namespace function
}  // namespace control
}  // namespace modules