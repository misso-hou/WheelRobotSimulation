#include "function/path_tracker.h"

#include <random>

#include "control_base.h"
#include "data_center.h"
#include "tools/math_tools.h"

namespace modules {
namespace control {
namespace function {

namespace mathTools = utilities::mathTools;

PathTracker::PathTracker() {
  //横向pid控制器
  lateral_pid_ptr_ = make_shared<alg::PID>();
  // 路径分类(摩擦系数,短直线阈值，点角度变化阈值，短曲线阈值,最小曲线速度阈值)
  path_classify_ptr_ = make_shared<alg::PathClassify>(0.01f, 0.01f, 1.2f, 3, 0.2f);
  // 纯跟踪参数设置(速度比例系数,轴距,最小预瞄距离约束)
  pp_ptr_ = make_shared<alg::PurePursuit>(1.5f, 0.91f, 0.35f);
  // stanley跟踪算法(横向误差比例系数\横向误差平滑系数\前后轮轴距(虚拟可调))
  stanley_ptr_ = make_shared<alg::StanleyController>(2.0f, 0.0f, 0.91f);
  // mpc控制器
  auto vehicle =
      make_shared<modules::vehicle::Unicycle>(base::k_cp_.min_omega, base::k_cp_.max_omega, -base::k_cp_.angular_acc, base::k_cp_.angular_acc);
  mpc_ptr_ = make_shared<alg::MPCController>(vehicle, 0.1f, 3.0f);
  //避障模块
  avoid_ = make_shared<AvoidCenter>();
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
  avoid_->SetTrackingPath(path_);
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
      port::CommonPose pose_ref_end = mathTools::Global2Rob(terminal_pose, base::k_robot_pose_);  // 将机器人位置转换到路径终点局部坐标系上,判断偏移量
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
 *   approach_switch:是否需要先到路径起点再对正方向
 *   accuracy_gain:到达精度增益
 *@return:角速度
 */
bool PathTracker::PathDirAlign(bool approach_switch, const float& accuracy_gain) {
  static bool local_first_flag = true;
  //如果已经完成路径起点对正,直接退出函数
  if (path_align_) {
    local_first_flag = true;
    return path_align_;
  }
  float angular = 0, linear = 0;
  //到达路径起点附近,并对正方向后再跟踪
  port::CommonPose goal = path_[0];
  float goal_angle = 0, angle_diff = 0;
  float goal_dist = mathTools::PointsDis(goal, base::k_robot_pose_);
  // 情况一:先到起点再对正方向
  if (approach_switch) {
    // 01->路径起点跟踪异常
    if (local_first_flag) {
      local_first_flag = false;
      goal.theta = mathTools::PointsAngle(goal, base::k_robot_pose_);
    }
    goal.theta = mathTools::PointsAngle(goal, base::k_robot_pose_);
    port::CommonPose pose_ref_end = mathTools::Global2Rob(goal, base::k_robot_pose_);
    if (pose_ref_end.x > 0.2f || fabs(pose_ref_end.y) > 1.0f) {
      approach_ = true;
    }
    // 02->靠近路径起点
    if (goal_dist > accuracy_gain * base::k_ct_.arrival_accuracy_thd && !approach_) {
      goal_angle = mathTools::PointsAngle(goal, base::k_robot_pose_) - base::k_robot_pose_.theta;
      if (base::k_task_.ref_cmd.linear >= 0) {
        goal_angle = mathTools::NormalizeAngle(goal_angle);
      } else {
        goal_angle = mathTools::NormalizeAngle(goal_angle + M_PI);
      }
      angular = goal_angle;
      if (goal_dist > 2) {
        linear = 2.0f;
      } else {
        linear = 0.3f;
      }
      if (fabs(goal_angle) > M_PI / 6) linear = 0.0f;
    }
    // 03->到达路径起点,对正路径方向
    else {
      approach_ = true;
      goal_angle = mathTools::PointsAngle(path_[1], path_[0]);
      if (base::k_task_.ref_cmd.linear >= 0) {
        angle_diff = mathTools::NormalizeAngle(goal_angle - base::k_robot_pose_.theta);
      } else {
        angle_diff = mathTools::NormalizeAngle(goal_angle - base::k_robot_pose_.theta + M_PI);
      }
      linear = 0.0f;
    }
    // 对准路径起始方向判断
    if (approach_) {
      if (fabs(angle_diff) > accuracy_gain * mathTools::DegToRad(base::k_ct_.angle_align_thd) + 0.1f) {
        angular = angle_diff;
      } else {
        approach_ = false;
        path_align_ = true;
      }
    }
  }
  // 情况二:无需到达起点附近，只对正方向
  else {
    goal_angle = mathTools::PointsAngle(path_[1], path_[0]);
    if (base::k_task_.ref_cmd.linear >= 0) {
      angle_diff = mathTools::NormalizeAngle(goal_angle - base::k_robot_pose_.theta);
    } else {
      angle_diff = mathTools::NormalizeAngle(goal_angle - base::k_robot_pose_.theta + M_PI);
    }
    // 对正方向
    if (fabs(angle_diff) > accuracy_gain * mathTools::DegToRad(base::k_ct_.angle_align_thd) + 0.1f) {
      angular = angle_diff;
      linear = 0.0f;
    } else {
      path_align_ = true;
    }
  }
  //速度平滑
  base::k_cmd_.linear = base::VP_Instance->LinearSpeedSmooth(linear);
  base::k_cmd_.angular = base::VP_Instance->AngularSpeedSmooth(angular);
  return path_align_;
}

void PathTracker::LinearSpeedPlanning() {
  nearest_index_ = mathTools::CalNearestPointIndexOnPath(path_, base::k_robot_pose_, base::k_goal_index_, 50);
  // tie(proximate_dg_border_,proximate_sharp_curve_) = path_classify_ptr_->GetProximateSpecialPath(nearest_index_);
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

float PathTracker::Avoid() {
  /***step01->正常跟踪期望速度***/
  Eigen::Vector2f ref_v;
  ref_v(0) = base::k_cmd_.linear;
  ref_v(1) = pp_ptr_->PurePursuitControl(base::k_robot_pose_, base::k_cmd_.linear, path_, base::k_goal_index_);
  /***step02->避障模块调用***/
  avoid_->SetRealTimeInfo(ref_v, base::k_goal_index_);
  base::k_goal_index_ = avoid_->GetAvoidGoalId();
  auto avoid_cmd = avoid_->GetAvoidCmd();
  base::k_cmd_.linear = base::VP_Instance->LinearSpeedSmooth(avoid_cmd.linear);
  return avoid_cmd.angular;
}

void PathTracker::ToPoseCtrl() {
  //[-TEST-]
  //虚拟加速度计算
  vec2f random_acc = vec2f::Random();
  static port::CommonPose virtual_pose = port::CommonPose(base::k_robot_pose_.x + 0.3f, base::k_robot_pose_.y, base::k_robot_pose_.theta);
  float dt = 0.02f;                      //系统tick时间
  random_acc(0) = random_acc(0) * 0.9f;  // 0.8线加速度
  random_acc(1) = random_acc(1) * 0.6f;  // 1.0角加速度
  static vec2f virtual_cmd;
  //虚拟速度生成
  virtual_cmd = virtual_cmd + dt * random_acc;
  virtual_cmd(0) = std::max(-0.0f, std::min(0.8f, virtual_cmd(0)));
  virtual_cmd(1) = std::max(-0.5f, std::min(0.5f, virtual_cmd(1)));
  std::random_device rd;
  std::mt19937 gen(rd());
  std::normal_distribution<float> noise(0.0f, 0.0f);  // 0.004
  float pose_noise = noise(gen);
  pose_noise = std::max(-0.02f, std::min(0.02f, pose_noise));
  virtual_pose.x += virtual_cmd(0) * cos(virtual_pose.theta) * dt + pose_noise;
  virtual_pose.y += virtual_cmd(0) * sin(virtual_pose.theta) * dt + pose_noise;
  virtual_pose.theta += virtual_cmd(1) * dt;
  port::Twist target_cmd = port::Twist(virtual_cmd(0), virtual_cmd(1));
  auto vel_cmd = avoid_->PoseControl(base::k_robot_pose_, virtual_pose, target_cmd);
  base::k_cmd_.linear = base::VP_Instance->LinearSpeedSmooth(vel_cmd.linear);
  base::k_cmd_.angular = base::VP_Instance->AngularSpeedSmooth(vel_cmd.angular);
}

void PathTracker::LateralControl() {
  float omega;
  switch (method_) {
    case CtrlALG::PP: {
      omega = pp_ptr_->PurePursuitControl(base::k_robot_pose_, base::k_cmd_.linear, path_, base::k_goal_index_);
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
    case CtrlALG::DYM_SYS: {
      omega = Avoid();
      break;
    }
    default:
      break;
  }
  if (method_ != CtrlALG::TO_POSE) {
    base::k_cmd_.angular = base::VP_Instance->AngularSpeedSmooth(omega);
    base::k_target_ = path_[base::k_goal_index_];
  }
}

port::TrackingInternalState PathTracker::PathTracking() {
  /*step00->路径处理*/
  if (base::k_first_) {
    base::k_first_ = false;
    PathDispose();
  }
  /*step01->到达判断*/
  if (ArrivedCheck(true, 5)) {
    auto state = base::Stop();  // 控制停机+获取停机状态
    return state;
  }
  /*step02->对齐路径起点*/  // TODO：参数调整函数调整
  if (!PathDirAlign(true, 2.0f)) {
    base::k_target_ = path_[0];
    return port::TrackingInternalState::TRACKING;
  }
  /*step03->线速度规划*/
  LinearSpeedPlanning();  // NOTE:注意，如果使用qp速度规划失败的补救措施,误差过大的处理方式
  /*step04->跟踪控制*/
  LateralControl();
  return port::TrackingInternalState::TRACKING;
}

}  // namespace function
}  // namespace control
}  // namespace modules