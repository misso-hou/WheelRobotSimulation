#pragma once

#include <algorithm>
#include <iostream>

#include "common_port/data_port.h"
#include "facilities/singleton.h"

namespace port = utilities::port;

namespace modules {
namespace control {
namespace algorithm {

class VelocityPlanner : public utilities::Singleton<VelocityPlanner> {
  friend class Singleton<VelocityPlanner>;

 public:
  VelocityPlanner(){};
  ~VelocityPlanner(){};

 public:
  void Init(port::ControlParam cp, port::CtrlThreshold ct);
  /*通用参数传递*/
  void SetVelPlanningParam(const port::CommonPose& pose, const port::Twist& last_cmd, const port::ActionState& state, const float& slope,
                           const port::RefCmd& reference_cmd);
  /*路径参数传递*/
  void SetPathData(const vector<port::CommonPose>& path, port::PathCurve nearest_curve = port::PathCurve{},
                   port::DangerousBorder nearest_dg_border = port::DangerousBorder{});
  /*纵向控制->线速度规划*/
  float VelocityPlanning(const float& ref_v, const int goal_index = 0, const float heading_error = 0.0f, const float lateral_error = 0.0f,
                         const int straight_path_flag = false);
  /*速度分配*/
  float VelocityAllot(const vector<port::TrajPoint>& traj_path, const int closest_point_index, const float& heading_error,
                      const float& lateral_error);
  /*角速度平滑*/
  float AngularSpeedSmooth(float omega);
  /*线速度平滑*/
  float LinearSpeedSmooth(float option_v);
  /*QP速度规划*/
  int SpeedProfilePlanning(vector<port::TrajPoint>& trajectory, const float& max_v);

  //速度规划配置参数(内部依赖)
 private:
  port::ControlParam ctrl_param_;  //控制配置参数
  port::CtrlThreshold ctrl_thd_;   //控制阈值参数
  float vel_planner_acc_;          //线加速度
  float vel_planner_dec_;          //线减速度

  //速度规划依赖参数(传入)
 private:
  //必要参数
  port::CommonPose robot_pose_;     //位姿
  float slope_param_;               //斜坡参数
  port::ActionState action_state_;  //异常状态
  port::Twist last_cmd_;            //上一时刻速度
  //路径参数
  bool path_flag_;                           //是否参考路径信息规划速度标志位
  vector<port::CommonPose> path_;            //路径数据
  port::PathCurve nearest_curve_;            //距离最近的弯道路径
  port::DangerousBorder nearest_dg_border_;  //距离最近的危险边界
};
}  // namespace algorithm
}  // namespace control
}  // namespace modules
