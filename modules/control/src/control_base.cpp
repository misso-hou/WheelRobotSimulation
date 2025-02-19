#include "control_base.h"

#include <iostream>

#include "algorithm/track/pid.h"
#include "algorithm/velocity_plan/velocity_planning.h"
#include "common_port/data_port.h"

namespace modules {
namespace control {
namespace base {

datacenter::DataCenter* DC_Instance = datacenter::DataCenter::GetInstance();
algorithm::VelocityPlanner* VP_Instance = algorithm::VelocityPlanner::GetInstance();

//控制器约束参数
port::ControlParam k_cp_;   //控制参数
port::CtrlThreshold k_ct_;  //控制约束阈值
//控制模块公共参数
port::CommonPose k_robot_pose_;
port::CommonPose k_target_;
int k_goal_index_;
port::Twist k_cmd_;
port::Twist k_last_cmd_;
port::TrackingTask k_task_;
//标志位变量
bool k_first_;
bool k_finish_;
bool k_csv_switch_;

//基础控制函数
void ResetBasicCtrl(const port::TrackingTask& new_task) {
  k_task_ = new_task;
  k_first_ = true;
  k_finish_ = false;
  k_last_cmd_ = port::Twist();
  k_target_ = port::CommonPose();
  k_goal_index_ = 0;
}

void SetRealTimeInfo() {
  k_robot_pose_ = DC_Instance->GetRobotPose();
  VP_Instance->SetVelPlanningParam(k_robot_pose_, k_last_cmd_, port::ActionState::NORMAL, 1.0f, k_task_.ref_cmd);
}

port::TrackingInternalState Stop() {
  /*停车速度平滑*/
  // k_cmd_.linear = VP_Instance->VelocityPlanning(0.0f);
  k_cmd_.linear = VP_Instance->LinearSpeedSmooth(0.0f);
  k_cmd_.angular = VP_Instance->AngularSpeedSmooth(0.0f);
  /*状态判断*/
  if (k_last_cmd_.angular == 0.0f && k_last_cmd_.linear == 0.0f) {
    float diff_x = fabs(k_robot_pose_.x - last_pose_.x);
    float diff_y = fabs(k_robot_pose_.y - last_pose_.y);
    float diff_theta = fabs(k_robot_pose_.theta - last_pose_.theta);
    if ((diff_x < 0.01f && diff_y < 0.01f && diff_theta < 0.1) || k_finish_) {
      k_finish_ = true;
      return port::TrackingInternalState::SUCCESS;
    }
  }
  return port::TrackingInternalState::TRACKING;
}

void UpdateData() {
  k_last_cmd_.angular = k_cmd_.angular;
  k_last_cmd_.linear = k_cmd_.linear;
  DC_Instance->SetCmdVel(k_cmd_);
  DC_Instance->SetTargetPose(k_target_);
  DC_Instance->SetCsvSwitch(k_csv_switch_);
}

}  // namespace base
}  // namespace control
}  // namespace modules
