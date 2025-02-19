
#include "algorithm/velocity_plan/velocity_planning.h"

#include <Eigen/Dense>

#include "OsqpEigen/OsqpEigen.h"

namespace modules {
namespace control {
namespace algorithm {

using namespace std;

void VelocityPlanner::Init(port::ControlParam cp, port::CtrlThreshold ct) {
  ctrl_param_ = cp;
  ctrl_thd_ = ct;
}

/**
 * @brief:设置速度规划计算相关参数(必要参数)
 * @param:
 *      pose: 机器人位姿
 *      last_cmd: 上一时刻速度
 *      state: 异常状态
 *      slope: 斜坡数据
 */
void VelocityPlanner::SetVelPlanningParam(const port::CommonPose& pose, const port::Twist& last_cmd, const port::ActionState& state,
                                          const float& slope, const port::RefCmd& reference_cmd) {
  robot_pose_ = pose;
  last_cmd_ = last_cmd;
  action_state_ = state;
  slope_param_ = slope;
  // 设置加/减速度(外部定义为第一优先级,固定参数次之)
  vel_planner_acc_ = reference_cmd.acceleration == 0.0f ? ctrl_param_.acceleration : reference_cmd.acceleration;
  vel_planner_dec_ = reference_cmd.deceleration == 0.0f ? ctrl_param_.deceleration : reference_cmd.deceleration;
}

/**
 * @brief:路径参数传递(非必要参数)
 * @param:
 *      pose: 路径跟踪(用于终点减速)
 *      nearest_curve: 距离最近的弯道(用于弯道减速)
 *      nearest_dg_border: 距离最近的危险边界(用于危险边界约束线速度)
 */
void VelocityPlanner::SetPathData(const vector<port::CommonPose>& path, port::PathCurve curve, port::DangerousBorder dg_border) {
  path_flag_ = true;
  path_ = path;
  nearest_curve_ = curve;
  nearest_dg_border_ = dg_border;
}

/**
 * @brief:实时速度分配
 * @param:
 *      goal_index:目标点对应路径索引(判断是否真正接近终点+判断不在起点处)
 *      heading_error:航向误差
 *      lateral_error:横向误差
 * @return:规划线速度
 */
float VelocityPlanner::VelocityAllot(const vector<port::TrajPoint>& traj_path, const int closest_point_index, const float& heading_error,
                                     const float& lateral_error) {
  vector<port::TrajPoint> copy_traj_path = traj_path;
  float cmd_linear_v;  //下发线速度
  /*场景速度:接近终点速度,异常状态处理速度,曲线跟踪速度,斜坡速度*/
  float traj_ref_v, state_v, slope_v, error_limit_v;
  traj_ref_v = state_v = slope_v = error_limit_v = copy_traj_path[closest_point_index].ref_v;
  /***[线速度规划01]-->异常状态速度模型***/
  if (action_state_ == port::ActionState::SLOWDOWN) {
    state_v = ctrl_param_.min_arround_v;  //异常状态速度设置
  } else if (action_state_ == port::ActionState::SHUTDOWN) {
    state_v = 0.0f;
    vel_planner_dec_ = ctrl_param_.deceleration;
  }
  /*确定规划速度*/
  float option_v = min(error_limit_v, min(slope_v, min(traj_ref_v, state_v)));
  //速度平滑处理
  cmd_linear_v = LinearSpeedSmooth(option_v);
  return cmd_linear_v;
}

/**
 * @brief:角速度平滑
 * @param:
 *      omega:横向控制期望角速度
 * @return:平滑后的角速度
 */
float VelocityPlanner::AngularSpeedSmooth(float omega) {
  //角速度限幅
  float limit_omega = max(ctrl_param_.min_omega, min(ctrl_param_.max_omega, omega));
  float smooth_omega = limit_omega;
  // 约束角速度增量
  if (fabs(limit_omega - last_cmd_.angular) > ctrl_param_.tick_time * ctrl_param_.angular_acc) {
    smooth_omega = (limit_omega - last_cmd_.angular > 0.0)
                       ? (last_cmd_.angular + ctrl_param_.tick_time * ctrl_param_.angular_acc)
                       : (last_cmd_.angular - ctrl_param_.tick_time * ctrl_param_.angular_acc);  //判断加速or减速,加/减速度增量
  }
  return smooth_omega;
}

/**
 * @brief:线速度平滑
 * @param:
 *      option_v:规划线速度
 *      ctrl_param:机器控制物理参数(加/减速度)
 * @return:跟踪速度
 */
float VelocityPlanner::LinearSpeedSmooth(float option_v) {
  float smooth_v;
  // 速度平滑处理
  smooth_v = option_v;
  float v_increment, v_decrement;
  v_increment = ctrl_param_.tick_time * vel_planner_acc_;
  v_decrement = ctrl_param_.tick_time * vel_planner_dec_;
  // 加速度约束
  if (option_v - last_cmd_.linear > v_increment) {
    smooth_v = last_cmd_.linear + v_increment;
  }
  // 减速度约束
  if (option_v - last_cmd_.linear < v_decrement) {
    smooth_v = last_cmd_.linear + v_decrement;
  }
  return smooth_v;
}

int VelocityPlanner::SpeedProfilePlanning(vector<port::TrajPoint>& trajectory, const float& max_v) {
  // QP元素矩阵定义
  Eigen::SparseMatrix<float> linearMatrix;
  Eigen::SparseMatrix<float> hessian;
  Eigen::VectorXd gradient;
  Eigen::VectorXd lowerBound;
  Eigen::VectorXd upperBound;
  // QP问题转化
  int N = trajectory.size();
  linearMatrix.resize((N - 1) + N, N);  // 加速度约束+速度约束系数矩阵
  // linearMatrix.setZero();
  // step01->不等式约束
  Eigen::VectorXd v_min = Eigen::VectorXd::Constant(N, 0.0);
  Eigen::VectorXd v_max = Eigen::VectorXd::Constant(N, 0.0);
  double acc_ratio = max(-1.0f, min(1.0f, 0.6f / max_v));  // 加速度比例约束
  Eigen::VectorXd acc_min = Eigen::VectorXd::Constant(N - 1, acc_ratio * ctrl_param_.deceleration);
  Eigen::VectorXd acc_max = Eigen::VectorXd::Constant(N - 1, acc_ratio * ctrl_param_.acceleration);
  // 构造加速度约束比例矩阵
  for (int i = 0; i < N - 1; i++) {
    port::TrajPoint current_point = trajectory[i];
    port::TrajPoint next_point = trajectory[i + 1];
    double delta_s = hypotf(next_point.pose.x - current_point.pose.x, next_point.pose.y - current_point.pose.y);
    linearMatrix.insert(i, i) = -1.f / (2.f * delta_s);  // 矩阵有N-1行,行索引最大到N-2
    linearMatrix.insert(i, i + 1) = 1.f / (2.f * delta_s);
    // 速度约束
    v_max[i] = i > 0 ? current_point.ref_v : 0.1;
  }
  // 线速度约束比例矩阵
  int j = 0;
  for (int i = N - 1; i < linearMatrix.rows(); i++) {
    linearMatrix.insert(i, j) = 1.f;
    j++;
  }
  // step02->cost function矩阵计算
  hessian.resize(N, N);
  hessian.setIdentity();
  gradient = -1 * v_max;
  lowerBound.resize(acc_min.size() + v_max.size());
  lowerBound << acc_min, v_min;
  upperBound.resize(acc_max.size() + v_max.size());
  upperBound << acc_max, v_max;
  // step03->调用QP求解器
  OsqpEigen::Solver solver;
  solver.settings()->setWarmStart(true);
  // set the initial data of the QP solver
  // solver.settings()->setVerbosity(false);
  solver.data()->setNumberOfVariables(N);
  solver.data()->setNumberOfConstraints(linearMatrix.rows());
  if (!solver.data()->setHessianMatrix(hessian)) return 1;
  if (!solver.data()->setGradient(gradient)) return 1;
  if (!solver.data()->setLinearConstraintsMatrix(linearMatrix)) return 1;
  if (!solver.data()->setLowerBound(lowerBound)) return 1;
  if (!solver.data()->setUpperBound(upperBound)) return 1;
  // instantiate the solver
  if (!solver.initSolver()) return 1;
  // controller input and QPSolution vector
  Eigen::VectorXd QPSolution;
  // solve the QP problem
  if (!solver.solve()) return 1;
  // get the controller input
  QPSolution = solver.getSolution();
  for (int j = 0; j < QPSolution.size(); j++) {
    trajectory[j].ref_v = QPSolution[j];
  }
  return 0;
}

}  // namespace algorithm
}  // namespace control
}  // namespace modules
