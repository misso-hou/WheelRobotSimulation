#include "algorithm/track/mpc.h"

#include "OsqpEigen/OsqpEigen.h"
#include "data_center.h"
#include "facilities/rate_controller.h"
#include "tools/math_tools.h"

/*
* QP问题设计：
*方式一：MPC转QP问题，优化变量为 horizon内的控制变量U
说明：套用MPC extend state space转QP标准公式,代码量少，内部计算较复杂抽象
*方式二: QP优化变量为state error
* 说明：使用标准state space模型,分步迭代计算,较直观，相对好理解
*/
namespace mathTools = utilities::mathTools;

namespace modules {
namespace control {
namespace algorithm {

datacenter::DataCenter* DC_Instance = datacenter::DataCenter::GetInstance();
/*
 * @brief:MPC控制器初始化
 * @param:
 *    model:被控对象模型
 *	  Q：被控对象State权重系数
 *	  Q_terminal:被控对象最终预测状态权重
 *	  R:被控对象input权重
 *	  Ts:预测采样时间
 *    horizon_time:预测时间
 */
MPCController::MPCController(const std::shared_ptr<vehicle::Unicycle>& model, const float& Ts, const float& horizon_time) {
  // todo:
  Q_.resize(2);
  Q_[0] = 1.0;
  Q_[1] = 1.0;
  R_.resize(1);
  R_[0] = 1.0;
  //被控对象信息获取
  model_ = model;
  n_x_ = model_->GetStateNum();
  n_u_ = model->GetInputNum();
  auto limit_u = model_->GetInputLimit();
  min_u_ = limit_u.first;
  max_u_ = limit_u.second;
  auto limit_u_dot = model_->GetInputDotLimit();
  min_u_dot_ = limit_u_dot.first;
  max_u_dot_ = limit_u_dot.second;
  //控制器内部变量设置
  horizon_time_ = horizon_time;
  sample_time_ = Ts;
  horizon_steps_ = std::round(horizon_time_ / sample_time_);
  N_X_ = horizon_steps_ * n_x_;
  N_U_ = horizon_steps_ * n_u_;
  CalQPConstrain();  // 计算QP常数矩阵
  //求解器定义
  osqp_solver_init_flag_ = true;
  osqp_solver_ = std::make_unique<OsqpEigen::Solver>();
  //创建独立线程
  mpc_thread_ = std::thread([&]() { MpcThreadFun(); });
  first_lock_ = true;
}

MPCController::~MPCController() {
  if (mpc_thread_.joinable()) {
    mpc_thread_.join();
  }
}

void MPCController::SetRealTimeStateInfo(const Eigen::VectorXf& cur_x, int cur_ref_id, const port::CommonPose& cur_pose,
                                         const Eigen::VectorXd& cur_u) {
  lock_guard<mutex> lk(thread_lock_);
  cur_x_ = cur_x;
  cur_ref_id_ = cur_ref_id;
  cur_pose_ = cur_pose;
  cur_u_ = cur_u;
  mpc_enable_ = true;
  UpdateQPConstrain();
}

void MPCController::SetRefTrajectory(const vector<port::TrajPoint>& ref_trajectory) {
  ref_traj_ = ref_trajectory;
  no_solution_count_ = 0;  // 重置内部变量
  cur_ref_id_ = 0;
}

/*
 * @brief:MPC转QP问题(方式一：优化变量为horizon control input U)(note:忽略Y = C*X,默认C为单位单位矩阵
 */
void MPCController::CalQPCostFunction() {
  // extend state space matrix
  Eigen::SparseMatrix<float, Eigen::RowMajor> Aex(N_X_, n_x_);
  Eigen::SparseMatrix<float, Eigen::RowMajor> Bex(N_X_, N_U_);
  Eigen::SparseMatrix<float, Eigen::RowMajor> Fex(N_X_, N_X_);
  Eigen::SparseMatrix<float, Eigen::RowMajor> Wex(N_X_, 1);
  // cost extend matrix
  Eigen::SparseMatrix<float> Qex(N_X_, N_X_);
  Eigen::SparseMatrix<float> Rex(N_U_, N_U_);
  //预测区间extend矩阵计算
  for (int i = 0; i < horizon_steps_; i++) {
    port::TrajPoint ref_point = horizon_ref_[i];
    auto [Ad, Bd, Wd] = model_->TrackingErrorSS(ref_point.ref_v, ref_point.K, sample_time_);
    // populate extend state space transition matrix
    int X_index = i * n_x_;
    int U_index = i * n_u_;
    Eigen::SparseMatrix<float> Aex_row_block(n_x_, n_x_);
    Eigen::SparseMatrix<float> Bex_row_block(n_x_, N_U_);
    Eigen::SparseMatrix<float> Fex_row_block(n_x_, n_x_);
    if (i == 0) {
      Aex_row_block = Ad;
    } else {
      Aex_row_block = Ad * Aex.middleRows(X_index - n_x_, n_x_);
      Bex_row_block = Ad * Bex.middleRows(X_index - n_x_, n_x_);
      Fex_row_block = Ad * Fex.middleRows(X_index - n_x_, n_x_);
    }
    //非对角线元素设置
    Aex.middleRows(X_index, n_x_) = Aex_row_block;
    Bex.middleRows(X_index, n_x_) = Bex_row_block;
    Fex.middleRows(X_index, n_x_) = Fex_row_block;
    Wex.middleRows(X_index, n_x_) = Wd;
    // Bex, Fex, Qex对角矩阵块赋值
    for (int row_id = 0; row_id < n_x_; row_id++) {
      // Bex对角线矩阵(n_x_,n_u_)
      for (int B_col_id = 0; B_col_id < n_u_; B_col_id++) {
        Bex.insert(X_index + row_id, U_index + B_col_id) = Bd.coeff(row_id, B_col_id);
      }
      // Fex对角线矩阵(单位矩阵)
      Fex.insert(X_index + row_id, X_index + row_id) = 1.f;
      // Qex对角线矩阵(n_x_,n_u_)
      Qex.insert(X_index + row_id, X_index + row_id) = Q_[row_id];
    }
    // Rex对角矩阵块赋值
    for (int u_id = 0; u_id < n_u_; u_id++) {
      Rex.insert(U_index + u_id, U_index + u_id) = R_[u_id];
    }
  }
  // QP cost function构造
  hessian_matrix_ = Bex.transpose() * Qex * Bex;  //!!!:+Rex操作有问题
  hessian_matrix_ = hessian_matrix_ + Rex;
  gradient_ = ((Aex * cur_x_ + Fex * Wex).transpose() * Qex * Bex - U_ref_.transpose() * Rex).cast<double>();
}

/*
 * @brief:MPC转QP不等式约束方程(时不变常量)
 */
void MPCController::CalQPConstrain() {
  //控制量约束
  Eigen::VectorXd lower_u(N_U_), upper_u(N_U_);
  Eigen::VectorXd lower_u_dot(N_U_), upper_u_dot(N_U_);
  //不等式线性系数矩阵
  Eigen::SparseMatrix<float> linear_contrain_u(N_U_, N_U_);
  linear_contrain_u.setIdentity();
  Eigen::SparseMatrix<float> linear_constrain_udot(N_U_, N_U_);
  /*****预测区间extend矩阵计算*****/
  for (int i = 0; i < horizon_steps_; i++) {
    int U_block_index = i * n_u_;
    //控制量约束计算
    lower_u.segment(U_block_index, n_u_) = min_u_;
    upper_u.segment(U_block_index, n_u_) = max_u_;
    //约束相对当前控制量的变化率
    if (i == 0) {
      lower_u_dot.segment(0, n_u_) = min_u_;
      upper_u_dot.segment(0, n_u_) = max_u_;
      //控制量不等式线性系数矩阵计算
      for (int j = 0; j < n_u_; j++) {
        linear_constrain_udot.insert(0 + j, 0 + j) = 1.f;
      }
    }
    //时间域内的控制量变化率约束
    else {
      lower_u_dot.segment(U_block_index, n_u_) = min_u_dot_;
      upper_u_dot.segment(U_block_index, n_u_) = max_u_dot_;
      //控制量不等式线性系数矩阵计算
      for (int j = 0; j < n_u_; j++) {
        linear_constrain_udot.insert(U_block_index + j, U_block_index - n_u_ + j) = -1 / sample_time_;
        linear_constrain_udot.insert(U_block_index + j, U_block_index + j) = 1 / sample_time_;
      }
    }
  }
  //不等式约束构造
  int constrian_n = 2 * N_U_;
  constraint_matrix_.resize(constrian_n, N_U_);
  constraint_matrix_.topRows(N_U_) = linear_contrain_u;
  constraint_matrix_.bottomRows(N_U_) = linear_constrain_udot;
  lower_bound_.resize(constrian_n);
  lower_bound_ << lower_u, lower_u_dot;
  upper_bound_.resize(constrian_n);
  upper_bound_ << upper_u, upper_u_dot;
}

/*
 * @brief:更新QP约束
 */
void MPCController::UpdateQPConstrain() {
  int insert_index = (horizon_steps_)*n_u_;
  lower_bound_.segment(insert_index, n_u_) = min_u_dot_ * sample_time_ + cur_u_;
  upper_bound_.segment(insert_index, n_u_) = max_u_dot_ * sample_time_ + cur_u_;
}

/*
 * @brief:计算预测区间内的reference state and input
 */
void MPCController::CalHorizonRef() {
  horizon_ref_.clear();
  horizon_ref_.resize(horizon_steps_);
  U_ref_.resize(horizon_steps_);
  float horizon_T = ref_traj_[cur_ref_id_].T;
  int horizon_ref_id = cur_ref_id_;
  // todo:horizon内的期望状态显示
  vector<port::CommonPose> ref_pose(horizon_steps_);
  for (int i = 0; i < horizon_steps_; i++) {
    horizon_T += sample_time_;
    //最临近插值->计算时间最近reference point
    for (int j = horizon_ref_id; j < ref_traj_.size(); j++) {
      if (horizon_T > ref_traj_[j].T) continue;
      //路径终点处理
      if (j == ref_traj_.size() - 1) {
        horizon_ref_id = j;
        break;  //防止后续计算越界
      }
      //计算时间最临近插值点
      float front_diff = horizon_T - ref_traj_[j - 1].T;
      float back_diff = ref_traj_[j].T - horizon_T;
      if (front_diff > back_diff) {
        horizon_ref_id = j;
      } else {
        horizon_ref_id = j - 1;
      }
      break;
    }
    horizon_ref_[i] = ref_traj_[horizon_ref_id];
    //规划路径期望角速度计算 ///???：是否需要在这里约束角速度
    U_ref_[i] = ref_traj_[horizon_ref_id].ref_v * ref_traj_[horizon_ref_id].K;
  }
}

/*
 * @brief:MPC对应的QP问题求解 //todo:数据格式问题
 */
int MPCController::SolveQP() {
  if (osqp_solver_init_flag_) {
    osqp_solver_init_flag_ = false;
    osqp_solver_ = unique_ptr<OsqpEigen::Solver>(new OsqpEigen::Solver());
    osqp_solver_->settings()->setWarmStart(true);
    // set the initial data of the QP solver
    osqp_solver_->settings()->setVerbosity(false);
    osqp_solver_->data()->setNumberOfVariables(N_U_);
    osqp_solver_->data()->setNumberOfConstraints(constraint_matrix_.rows());
    if (!osqp_solver_->data()->setHessianMatrix(hessian_matrix_)) return 1;
    if (!osqp_solver_->data()->setGradient(gradient_)) return 1;
    if (!osqp_solver_->data()->setLinearConstraintsMatrix(constraint_matrix_)) return 1;
    if (!osqp_solver_->data()->setLowerBound(lower_bound_)) return 1;
    if (!osqp_solver_->data()->setUpperBound(upper_bound_)) return 1;
    // instantiate the solver
    if (!osqp_solver_->initSolver()) return 1;
    if (!osqp_solver_->solve()) return 1;
  } else {
    // solve the QP problem
    if (!osqp_solver_->updateHessianMatrix(hessian_matrix_)) return 1;
    if (!osqp_solver_->updateGradient(gradient_)) return 1;
    if (!osqp_solver_->updateBounds(lower_bound_, upper_bound_)) return 1;
    // if (!osqp_solver_->updateLinearConstraintsMatrix(constraint_matrix_)) return 1;
    if (!osqp_solver_->solve()) return 1;
  }
  qp_solution_ = osqp_solver_->getSolution();
  return 0;
}

/*
 * @brief:控制器调用
 */
void MPCController::MPCcontrol() {
  {
    lock_guard<mutex> lk(thread_lock_);
    /***step01->计算预测时域内的reference***/
    CalHorizonRef();
    /***step02->计算QP cost function***/
    CalQPCostFunction();
  }
  /***step03->QP求解&&异常处理***/
  {
    lock_guard<mutex> lk(solution_thread_lock_);
    if (SolveQP() == 1) {
      no_solution_count_ += 1;
      int local_index = no_solution_count_ * n_u_;
      mpc_output_ = qp_solution_[local_index];
      // todo:计算预测轨迹
    } else {
      no_solution_count_ = 0;
      // get the controller output
      mpc_output_ = qp_solution_[0];
      vector<port::CommonPose> predict_pose(qp_solution_.size());
      port::CommonPose next_pose = cur_pose_;
      for (int i = 0; i < qp_solution_.size(); i++) {
        next_pose = model_->StateUpdate(next_pose, horizon_ref_[i].ref_v, qp_solution_[i], sample_time_);
        predict_pose[i] = next_pose;
      }
      DC_Instance->SetMpcHorizon(predict_pose);
    }
  }
}

/*
 * @brief:控制器独立线程
 */
int32_t MPCController::MpcThreadFun() {
  utilities::facilities::RateController rt{SIMULATION_RATE * 10.0};
  while (1) {
    if (mpc_enable_) {
      MPCcontrol();
    }
    rt.Spin();
  }
  return 0;
}
}  // namespace algorithm
}  // namespace control
}  // namespace modules
