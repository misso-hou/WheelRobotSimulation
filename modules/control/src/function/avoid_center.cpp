#include "function/avoid_center.h"

#include "control_base.h"
#include "facilities/rate_controller.h"

namespace modules {
namespace control {
namespace function {

namespace alg = algorithm;
namespace ab = alg::avoid_base;

AvoidCenter::AvoidCenter() {
  InitAvoidbase();
  //算法工具
  avt_ptr_ = make_shared<alg::AvoidTracking>();
  dw_obs_ptr_ = make_shared<alg::DealWithObs>(5, 2.5f, 0.05f);
  dyn_sys_ptr_ = make_shared<alg::DynamicSys>(0.2f, 7, 1.0f, 2.0f);
  //线程
  avoid_thread_ = std::thread([&]() { AvoidThreadFun(); });
}

void AvoidCenter::InitAvoidbase() {
  //动态系统算法配置
  port::SampleCircle sample_circle;  //障碍采样轮廓(中心圆)
  sample_circle.sub_num = 60;
  sample_circle.range = 3.0f;
  sample_circle.center_x = 0.5f;
  sample_circle.center_y = 0.0f;
  sample_circle.radius = 1.0f;
  // 避障轮廓设置
  // vector<port::BoundaryCircle> avoid_circles(1);
  // avoid_circles[0].radius = 0.81f;
  // avoid_circles[0].center_x = 0.48f;
  // avoid_circles[0].center_y = 0.0f;
  // avoid_circles[0].safe_margin = 0.1f;
  vector<port::BoundaryCircle> avoid_circles(2);
  avoid_circles[0].radius = 0.665f;
  avoid_circles[0].center_x = 0.68f;
  avoid_circles[0].center_y = 0.0f;
  avoid_circles[0].safe_margin = 0.2f;
  avoid_circles[1].radius = 0.665f;
  avoid_circles[1].center_x = 0.3f;
  avoid_circles[1].center_y = 0.0f;
  avoid_circles[1].safe_margin = 0.2f;
  // 障碍物处理方法
  ab::Method method = ab::Method::POLYGON;
  ab::Init(sample_circle, avoid_circles, method);
}

port::Twist AvoidCenter::GetAvoidCmd() {
  std::lock_guard<std::mutex> lk(thread_lock_);
  return avoid_cmd_;
}

int AvoidCenter::GetAvoidGoalId() {
  std::lock_guard<std::mutex> lk(thread_lock_);
  return goal_id_;
}

void AvoidCenter::ShutdownAvoid() {
  avoid_enable_ = false;
  for (auto& agent : ab::g_agents) {
    agent.center_pose = vec2f(0, 0);
  }
  ab::g_goal_index = 0;
  ab::ResetAvoidBaseData();
  ab::UpdateInfo();
}

void AvoidCenter::SetRealTimeInfo(const vec2f& ref_vel, const int goal_id) {
  std::lock_guard<std::mutex> lk(thread_lock_);
  interface_pose_ = base::k_robot_pose_;
  interface_cur_cmd_(0) = base::k_last_cmd_.linear;
  interface_cur_cmd_(1) = base::k_last_cmd_.angular;
  interface_ref_cmd_ = ref_vel;
  interface_goal_id_ = goal_id;
  avoid_enable_ = true;
}

void AvoidCenter::SetTrackingPath(const vector<port::CommonPose>& path) {
  auto boundary = base::DC_Instance->GetBoundary();
  avt_ptr_->SetTrackingPath(path, boundary);
}

port::Twist AvoidCenter::PoseControl(const port::CommonPose& robot_pose, const port::CommonPose target_pose, const port::Twist target_cmd) {
  auto cmd = avt_ptr_->PoseCtrl(robot_pose, target_pose, target_cmd);
  return port::Twist(cmd(0), cmd(1));
}

void AvoidCenter::UpdateAvoidBaseData() {
  std::lock_guard<std::mutex> lk(thread_lock_);
  ab::g_robot_pose = interface_pose_;
  ab::g_obs_points = base::DC_Instance->GetMapObs();
  ab::g_cur_cmd = interface_cur_cmd_;
  ab::g_ref_cmd = interface_ref_cmd_;
  //清空历史数据(有些用于可视化，必须清空)
  ab::ResetAvoidBaseData();
  //避障速度赋初值(外部规划的跟踪速度)
  ab::g_avoid_cmd.linear = interface_ref_cmd_(0);
  ab::g_avoid_cmd.angular = interface_ref_cmd_(1);
  ab::g_goal_index = interface_goal_id_;
}

void AvoidCenter::SetInterfaceInfo() {
  std::lock_guard<std::mutex> lk(thread_lock_);
  goal_id_ = ab::g_goal_index;
  avoid_cmd_ = ab::g_avoid_cmd;
}

int32_t AvoidCenter::AvoidThreadFun() {
  utilities::facilities::RateController rt{SIMULATION_RATE * 20.0};  // 数据记录频率
  while (1) {
    if (avoid_enable_) {
      /***step01->更新实时数据***/
      UpdateAvoidBaseData();
      /***step02->障碍物处理(界内+虚拟障碍物)***/
      auto result = dw_obs_ptr_->DisposeObs(ab::g_obs_points);
      if (result != 0) {
        /***step03->agents速度矢量信息更新(&跟踪目标点更新)***/
        avt_ptr_->UpdateAgentsVelVecInfo(interface_goal_id_);
        /***step04->根据“计算障碍物”规划矢量速度模长***/
        float planning_v = avt_ptr_->CalVelocityEnvelope();
        /***step05->动态系统避障速度计算***/
        ab::g_avoid_cmd = dyn_sys_ptr_->DynamicSysAvoid(planning_v);
      }
      /***step05->更新避障模块信息***/
      SetInterfaceInfo();
      ab::UpdateInfo();
    }
    rt.Spin();
  }
  return 0;
}

}  // namespace function
}  // namespace control
}  // namespace modules