#pragma once

#include <iostream>

#include "algorithm/track/mpc.h"
#include "algorithm/track/pid.h"
#include "algorithm/track/pure_pursuit.h"
#include "algorithm/track/stanley.h"
#include "algorithm/velocity_plan/path_classify.h"
#include "function/avoid_center.h"

namespace modules {
namespace control {
namespace function {

namespace port = utilities::port;
namespace alg = modules::control::algorithm;

//控制算法选项
enum CtrlALG {
  PP = 0,   // 纯跟踪
  STANLEY,  // stanley跟踪
  P_I_D,    // PID直线控制器
  TO_POSE,  // 到点控制
  MPC,      // MPC横向控制
  DYM_SYS   // 动态系统算法
};

class PathTracker {

 public:
  PathTracker();
  ~PathTracker() {}

 public:
  void ResetPathTracker(const vector<port::CommonPose>& new_path, const CtrlALG& method);
  port::TrackingInternalState PathTracking();

 private:
  void PathDispose();
  bool ArrivedCheck(bool overtime_check, int dis_gain);
  bool PathDirAlign(bool approach_switch, const float& accuracy_gain);
  void LinearSpeedPlanning();
  float MpcTracking();
  float Avoid();
  void ToPoseCtrl();
  void LateralControl();

 private:
  shared_ptr<alg::PurePursuit> pp_ptr_;              //纯跟踪
  shared_ptr<alg::StanleyController> stanley_ptr_;   // stanley跟踪控制器
  shared_ptr<alg::PathClassify> path_classify_ptr_;  //路径分类
  shared_ptr<alg::MPCController> mpc_ptr_;           // mpc控制器
  shared_ptr<alg::PID> lateral_pid_ptr_;             //横向误差控制器
  shared_ptr<AvoidCenter> avoid_;                    //避障功能单元

 private:
  int goal_index_;                             //跟踪目标点索引
  int nearest_index_;                          //控制中心相对路径最近点
  vector<port::CommonPose> path_;              //单条跟踪路径path
  port::DangerousBorder proximate_dg_border_;  //距离机器最近的危险边界
  port::PathCurve proximate_sharp_curve_;      //距离机器最近的急转弯
  vector<port::TrajPoint> traj_path_;          //带规划速度路径
  CtrlALG method_;                             //算法选择
  //状态标志
  bool path_align_;  //路径开始跟踪前，方向对准状态锁
  bool approach_;    //接近终点标志
};

}  // namespace function
}  // namespace control
}  // namespace modules