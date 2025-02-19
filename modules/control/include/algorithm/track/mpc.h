#pragma once

#include <Eigen/Dense>
#include <Eigen/SparseCore>  //note:无法引用<Eigen/Sparse>,所以也无法引用OsqpEigen
#include <iostream>
#include <memory>
#include <mutex>
#include <thread>

#include "OsqpEigen/OsqpEigen.h"
#include "bycicle.h"
#include "common_port/data_port.h"
#include "unicycle.h"

namespace modules {
namespace control {
namespace algorithm {

using namespace std;
namespace port = utilities::port;
namespace vehicle = modules::vehicle;

class MPCController {
 public:
  MPCController(const std::shared_ptr<vehicle::Unicycle>& model, const float& Ts, const float& horizon_time);
  MPCController(const std::shared_ptr<vehicle::Bycicle>& model, const float& Ts, const float& horizon_time);
  ~MPCController();

 public:
  void SetRefTrajectory(const vector<port::TrajPoint>& ref_trajectory);
  void SetRealTimeStateInfo(const Eigen::VectorXf& cur_x, int cur_ref_id, const port::CommonPose& cur_pose, const Eigen::VectorXd& cur_u);
  void ShutdownMPC() {
    mpc_enable_ = false;
    osqp_solver_init_flag_ = true;
  }

  float GetMPCOutput() {
    lock_guard<mutex> lk(solution_thread_lock_);
    return mpc_output_;
  }

 private:
  //方式一:二次优化变量为预测区间内的控制量U
  void CalQPCostFunction();
  void CalQPConstrain();
  void CalHorizonRef();
  void UpdateQPConstrain();
  int SolveQP();
  void MPCcontrol();

 private:  // mpc内部变量
  std::unique_ptr<OsqpEigen::Solver> osqp_solver_;
  std::shared_ptr<vehicle::Unicycle> model_;
  //实时数据
  Eigen::VectorXf cur_x_;  //当前误差信息（横向误差+航向误差）
  int cur_ref_id_;         //当前最近点
  port::CommonPose cur_pose_;
  Eigen::VectorXd cur_u_;
  //控制器预设变量
  size_t n_x_;
  size_t n_u_;
  size_t N_X_;
  size_t N_U_;
  float sample_time_;
  float horizon_time_;
  size_t horizon_steps_;
  vector<port::TrajPoint> horizon_ref_;
  Eigen::VectorXf U_ref_;
  //权重系数矩阵
  Eigen::VectorXf Q_;
  Eigen::VectorXf R_;

 private:  // QP求解器变量
  // QP求解器cost function系数矩阵
  Eigen::SparseMatrix<float> hessian_matrix_;
  Eigen::VectorXd gradient_;
  //约束系数矩阵
  Eigen::SparseMatrix<float, Eigen::RowMajor> constraint_matrix_;
  Eigen::VectorXd lower_bound_;
  Eigen::VectorXd upper_bound_;
  //求解器
  // unique_ptr<OsqpEigen::Solver> osqp_solver_;
  bool osqp_solver_init_flag_ = false;  // note：无法在头文件中引用osqp-eigen头文件
  int no_solution_count_;               //计算异常计数
  Eigen::VectorXd qp_solution_;
  float mpc_output_;

 private:
  vector<port::TrajPoint> ref_traj_;  // 完整期望轨迹
  //机器物理约束
  Eigen::VectorXd min_u_;
  Eigen::VectorXd max_u_;
  Eigen::VectorXd min_u_dot_;
  Eigen::VectorXd max_u_dot_;

 private:  //独立线程相关变量
  int32_t MpcThreadFun();
  std::mutex thread_lock_;           // 线程数据传输锁
  std::mutex solution_thread_lock_;  // 线程数据传输锁
  std::thread mpc_thread_;
  bool mpc_enable_;
  bool first_lock_;
};
}  // namespace algorithm
}  // namespace control
}  // namespace modules
