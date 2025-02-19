#pragma once

#include <iostream>

#include "algorithm/track/pid.h"
#include "algorithm/velocity_plan/velocity_planning.h"
#include "common_port/data_port.h"
#include "data_center.h"

namespace modules {
namespace control {
namespace base {

namespace port = utilities::port;

extern datacenter::DataCenter* DC_Instance;
extern algorithm::VelocityPlanner* VP_Instance;  // velocity planner单例

//控制器约束参数
extern port::ControlParam k_cp_;   // 控制参数
extern port::CtrlThreshold k_ct_;  // 控制约束阈值
//控制模块公共参数
extern port::CommonPose k_robot_pose_;
extern port::CommonPose k_target_;
extern int k_goal_index_;
extern port::Twist k_cmd_;
extern port::Twist k_last_cmd_;  // 上一周期速度指令
extern port::TrackingTask k_task_;
//标志位变量
extern bool k_first_;
extern bool k_finish_;
extern bool k_csv_switch_;

//内部数据
static port::CommonPose last_pose_;  // 上一时刻定位

//功能函数
void ResetBasicCtrl(const port::TrackingTask& new_task);
void SetRealTimeInfo();
port::TrackingInternalState Stop();
void UpdateData();

}  // namespace base
}  // namespace control
}  // namespace modules
