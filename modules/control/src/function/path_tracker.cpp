#include "function/path_tracker.h"

#include <random>

#include "control_base.h"
#include "data_center.h"
#include "tools/math_tools.h"

namespace mathTools = utilities::mathTools;
namespace alg = algorithm;

namespace modules {
namespace control {
namespace function {

PathTracker::PathTracker() {
  //横向pid控制器
  lateral_pid_ptr_ = make_shared<alg::PID>();
  // 路径分类(摩擦系数,短直线阈值，点角度变化阈值，短曲线阈值,最小曲线速度阈值)
  path_classify_ptr_ = make_shared<alg::PathClassify>(0.01f, 0.01f, 1.2f, 3, 0.2f);
  // 纯跟踪参数设置(速度比例系数,轴距,最小预瞄距离约束)
  pp_ptr_ = make_shared<alg::PurePursuit>(0.0, 0.91, 0.4);
  // stanley跟踪算法(横向误差比例系数\横向误差平滑系数\前后轮轴距(虚拟可调))
  stanley_ptr_ = make_shared<alg::StanleyController>(2.0f, 0.0, 0.91f);
  // mpc控制器
  auto vehicle = make_shared<modules::vehicle::Unicycle>(base::k_cp_.min_omega, base::k_cp_.max_omega, -base::k_cp_.angular_acc);
  mpc_ptr_ = make_shared<alg::MPCController>(vehicle, 0.1f, 3.0f);
  //避障模块
  avoid_ = make_shared<avoidCenter>();
}

void PathTracker::ResetPathTracker(const vector<port::CommonPose>& new_path, const CtrlALG& method) {
  path_ = new_path;
  method_ = method;
  path_align_ = false;
  approach_ = false;
  nearest_index_ = 0;
  //关闭mpc
  mpc_ptr_->ShutdownMPC();
  //关闭动态系统
  avoid_->ShutdownAvoid();
}

void PathTracker::PathDispose() {
  /***step01->原始跟踪路径二次处理***/
  vector<port::CommonPose> display_sharp_curves, display_dg_borders;
  path_classify_ptr_->PathDispose(path_);
  auto [plt_borders, plt_curves] = path_classify_ptr_->GetPltSpecialPath();
  traj_path_ = path_classify_ptr_->GetPlanningTrajectory();
  /***step02->控制路径跟踪模块后处理路径上报***/
  base::DC_Instance->SetPathUpdateFlag(true);
  base::DC_Instance->SetPlanningPath(path_);
  base::DC_Instance->SetSpCurves(plt_curves);
  base::DC_Instance->SetDgBorders(plt_borders);
  base::DC_Instance->SetTrajectoryPath(traj_path_);
  //算法模块传参
  mpc_ptr_->SetRefTrajectory(traj_path_);
}

bool PathTracker::ArrivedCheck(bool overtime_check, int dis_gain) {
  bool flag = false;
  /*step01->路径跟踪完成判断*/
  port::CommonPose terminal_pose = path_.back();
  float diff_x = terminal_pose.x - base::k_robot_pose_.x;
  float diff_y = terminal_pose.y - base::k_robot_pose_.y;
  /*路径终点到达判断*/
  bool approach_flag = (fabs(diff_x) < base::k_ct_.arrival_accuracy_thd && fabs(diff_y) < base::k_ct_.arrival_accuracy_thd);
  /*step02->跟踪异常判断*/
  bool force_arrive_flag, abnormal_flag;
  force_arrive_flag = abnormal_flag = false;
  if (base::k_goal_index_ > ((int)path_.size() - 10) || (path_.size() == 1)) {  // 防止闭环路径导致的提前到达问题
    // 情况一:终点跟踪趋势与终点跟踪超时判断任务异常判断(路径太短，到点控制不判断超时)
    float path_end_dis = hypotf(diff_x, diff_y);
    if (path_end_dis < dis_gain * base::k_ct_.arrival_accuracy_thd) {
      // abnormal_flag = TaskAbnormalCheck(path_.back(), overtime_check);
      force_arrive_flag = true;
    }
    //情况二：根据终点坐标系判断异常
    if (path_.size() > 1) {
      terminal_pose.theta = mathTools::PointsAngle(terminal_pose, path_[path_.size() - 2]);
      port::CommonPose pose_ref_end = mathTools::Global2Rob(terminal_pose,
                                                            base::k_robot_pose_);  // 将机器人位置转换到路径终点局部坐标系上,判断便宜量
      if (pose_ref_end.x > 0.f || fabs(pose_ref_end.y) > 1.0f) {
        force_arrive_flag = true;
      }
    }
    /*step03->任务完成判断*/
    if (approach_flag || force_arrive_flag || abnormal_flag || base::k_finish_ == true) {
      base::k_finish_ = true;
      flag = true;
    } else {
      flag = false;
    }
  }
  return flag;
}

/*
 *@brief:路径跟踪开始方向对正
 *@param:
 *   approach_switch:是否先到路径起点再对正方向
 *   accuracy_gain:到达精度
 *@return 角速度
 */
bool PathTracker::PathDirAlign(bool approach_switch, const float& accuracy_gain) {}

void PathTracker::LinearSpeedPlanning() {
  nearest_index_ = mathTools::CalNearestPointIndexOnPath(path_, base::k_robot_pose_, base::k_goal_index_, 50);
  // tie(proximate_dg_border_,proximate_sharp_curve_) = path_classify_ptr_->GetProximateSpecialPath(ne
  port::CommonPose nearest_point = path_[nearest_index_];
  float lateral_error = mathTools::PointsDis(nearest_point, base::k_robot_pose_);
  float ref_dir = mathTools::PointsAngle(path_[base::k_goal_index_], base::k_robot_pose_);
  float heading_error = mathTools::NormalizeAngle(ref_dir - base::k_robot_pose_.theta);
  base::k_cmd_.linear = base::VP_Instance->VelocityAllot(traj_path_, nearest_index_, heading_error, lateral_error);
}

/**
 * @brief:mpc
 */
float PathTracker::MpcTracking() {
  base::k_goal_index_ = nearest_index_;
  port::CommonPose nearest_point = path_[nearest_index_];
  port::CommonPose trans_point = mathTools::Global2Rob(base::k_robot_pose_, nearest_point);
  float heading_error = mathTools::NormalizeAngle(traj_path_[nearest_index_].pose.theta - base::k_robot_pose_.theta);
  // mpc实时数据
  Eigen::Vector2f cur_state(2);
  cur_state[0] = trans_point.y;
  cur_state[1] = heading_error;
  // 防止初始误差导致的过修正(求解器首次计算有问题)
  if (nearest_index_ == 0) {
    cur_state[0] = 0;
    cur_state[1] = 0;
  }
  Eigen::VectorXd cur_input(1);
  cur_input[0] = base::k_last_cmd_.angular;
  mpc_ptr_->SetRealTimeStateInfo(cur_state, nearest_index_, base::k_robot_pose_, cur_input);
  float angular = mpc_ptr_->GetMPCOutput();
  return angular;
}

float PathTracker::DynSysAvoid() {}

void PathTracker::LateralControl() {
  float omega;
  switch (method_) {
    case CtrlALG::PP: {
      omega = pp_ptr_->PurePursuitControl(base::k_robot_pose_, base::k_cmd_.linear, base::k_task_.ref_cmd.linear, path_, base::k_goal_index_);
      break;
    }
    case CtrlALG::STANLEY: {
      omega = stanley_ptr_->StanleyControl(base::k_robot_pose_, path_, base::k_goal_index_, base::k_cmd_.linear, base::k_task_.ref_cmd.linear,
                                           base::k_task_.plan_type);
      break;
    }
    case CtrlALG::P_I_D: {
      break;
    }
    case CtrlALG::TO_POSE: {
      ToPoseCtrl();
      break;
    }
    case CtrlALG::MPC: {
      omega = MpcTracking();
      break;
    }
    case CtrlALG::DYM_SYS { omega = Avoid(); break; } default:
      break;
  }
  if (method_ != CtrlALG::TO_POSE) {
    base::k_cmd_.angular = base::VP_Instance->AngularSpeedSmooth(omega);
    base::k_target_ = path_[base::k_goal_index_];
  }
}

port::TrackingInternalState PathTracker::PathTracking() {}

}  // namespace function
}  // namespace control
}  // namespace modules