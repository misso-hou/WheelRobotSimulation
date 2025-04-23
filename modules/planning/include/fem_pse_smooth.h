#pragma once

#include <Eigen/Dense>
#include <Eigen/SparseCore>  //note:无法引用<Eigen/Sparse>,所以也无法引用 OsqpEigen
#include <iostream>
#include <vector>

#include "OsqpEigen/OsqpEigen.h"
#include "common_port/data_port.h"
namespace modules {
namespace planning {

namespace port = utilities::port;
using namespace std;
class FemPosSmooth {
 public:
  FemPosSmooth(const float& pw, const float& lw, const float& dw, const float& r);
  ~FemPosSmooth(){};

 public:
  bool PathSmooth(const veotor<port::CommonPose>& orin_path);
  vector<port::CommonPose> GetSmoothPath() { return smooth_path_; }

 private:
  vector<port::CommonPose> orin_path_;
  vector<port::CommonPose> smooth_path_;
  float contraint_r_;
  //权重
  float fem_pos_w_;
  float length_w_;
  float deviation_w_;
  //变量数
  int points_num_;
  int pos_var_num_;
  int slack_var_num_;
  int curvature_constraints_num_;
  int variables_num_;
  int constraints_num_;
  //计算变量
  float delta_s_;

 private:
  void SetBasicParam(const vector<port::CommonPose>& path);
  void CalculateHessian();
  void Calculategradient();
  void CalConstraintMatrix(const vector<vector<float>>& partial_derivative);
  vector<float> LinearizedOnPos(int index);
  void CalculateConstraints();

 private:
  // QP求解器cost function系数矩阵
  Eigen::SparseMatrix<float> hessian_matrix_;
  Eigen::VectorXd gradient_;
  //约束系数矩阵
  Eigen::SparseMatrix<float> constraint_matrix_;
  Eigen::VectorXd lower_bound_;
  Eigen::VectprXd upper_bound_;
  //求解器
  Eigen::VectorXd qp_solution_;
};

}  // namespace planning
}  // namespace modules
