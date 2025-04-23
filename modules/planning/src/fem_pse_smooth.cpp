#include "fem_pse_smooth.h"

#include <cmath>

namespace modules {
namespace planning {

FemPosSmooth::FemPosSmooth(const float &pw, const float &lw, const float &dw, const float &r) {
  //设置权重：曲度(向量模长法）+ 点距 + 位移
  fem_pos_w_ = pw;
  length_w_ = lw;
  deviation_w_ = dw;
  //最小转弯半径约束
  contraint_r_ = r;
  //求解器
  // osqp_solver_ = std::make_unique<OsqpEigen::Solver>();
}

/**
 *@breif:基础变量计算
 */
void FemPosSmooth::SetBasicParam(const vector<port::CommonPose> &path) {
  orin_path_ = path;
  points_num_ = orin_path_.size();
  pos_var_num_ = points_num_ * 2;
  slack_var_num_ = points_num_ - 2;
  //变量X向量size
  variables_num_ = pos_var_num_ + slack_var_num_;
  //约束
  curvature_constraints_num_ = points_num_ - 2;
  constraints_num_ = pos_var_num_ + slack_var_num_ + curvature_constraints_num_;
  //平均点距
  float total_length = 0.0;
  auto pre_point = orin_path_.front();
  for (uint i = 1; i < points_num_; ++i) {
    const auto &cur_point = points[i];
    total_length += std::hypotf(pre_point.x - cur_point.x, pre_point.y - cur_point.y);
    pre_point = cur_point;
  }
  delta_s_ = total_length / (points_num_ - 1);
}


/**
 *@breif:hessian矩阵计算
 */
void FemPosSmooth::CalculateHessian() {
  /*
  // General formulation of P matrix is as below(with 6 points as an example):
  // I is a two by two identity matrix, X, Y, Z represents x * I, y * I, z * I
  // 0 is a two by two zero matrix
  // |X+Y+Z, -2X-Y,   X,       0,       ...    0,       0,       ...|
  // |0,     5X+2Y+Z, -4X-Y,   X,       ...    0,       0,       ...|
  // |0,     0,       6X+2Y+Z, -4X-Y,   ...    0,       0,       ...|
  // |0,     0,       0,       6X+2Y+Z, ...    0,       X,       ...|
  // |...    ...      ...      ...      ...    ...      ...      ...|
  // |...    ...      ...      ...      ...    x,       0,       ...|
  // |...    ...      ...      ...      ...    -4X-Y,   x,       ...|
  // |0,     0,       0,       0,       ...    5X+2Y+Z, -2X-Y    ...|
  // |0,     0,       0,       0,       ...    0,       X+Y+Z    ...|
  // |...    ...      ...      ...      ...    ...      ...      ...|
  // |...    ...      ...      ...      ...    ...      ...      ...|
  * hessian矩阵按列元素进行划分可以分成3个部分，第1/2列，倒数第1/2列，中间所有列
  * note:osqp的高版本（0.6.0之后）要求P矩阵只写上三角部分就可以了
  */
  Eigen::MatrixXf hessian_dense(variables_num_, variables_num_);
  hessian_dense.setZero();
  /*step01->X,Y,Z基础矩阵计算*/
  Eigen::Matrix2f mx, my, mz;
  mx = fem_pos_w_ * Eigen::Matrixf::Identity();
  my = length_w_ * Eigen::Matrix2f::Identity();
  mz = deviation_w_ * Eigen::Matrix2f::Identity();
  /*step02->hessian矩阵按列分区域填值(左上block填值)*/
  for (uint i = 0; i < points_num_; ++i) {
    //第一列(实际是前两列,两列为一组)
    if (i == 0) {
      hessian_dense.block(0, 0, 2, 2) = mx + my + mz;
      continue;
    }
    //第二列
    if (i == 1) {
      hessian_dense.block(0, 2, 2, 2) = -2 * mx - my;
      hessian_dense.block(2, 2, 2, 2) = 5 * mx + 2 * my + mz;
      continue;
    }
    //中间列
    if (i < (points_num_ - 2)) {
      hessian_dense.block(2 * (i - 2), 2 * i, 2, 2) = mx;
      hessian_dense.block(2 * (i - 1), 2 * i, 2, 2) = -4 * mx - my;
      hessian_dense.block(2 * i, 2 * i, 2, 2) = 6 * mx + 2 * my + mz;
      continue;
    }
    //倒数第二列
    if (i == (points_num_ - 2)) {
      hessian_dense.block(2 * (i - 2), 2 * i, 2, 2) = mx;
      hessian_dense.block(2 * (i - 1), 2 * i, 2, 2) = -4 * mx - my;
      hessian_dense.block(2 * i, 2 * i, 2, 2) = 5 * mx + 2 * my + mz;
      continue;
    }
    //最后一列
    if (i == (points_num_ - 1)) {
      hessian_dense.block(2 * (i - 2), 2 * i, 2, 2) = mx;
      hessian_dense.block(2 * (i - 1), 2 * i, 2, 2) = (-2 * mx - my);
      hessian_dense.block(2 * i, 2 * i, 2, 2) = (mx + my + mz);
      continue;
    }
  }
  hessian_matrix_ = hessian_dense.sparseView();
}

/**
 *@breif:gradient矩阵计算
 */
void FemPosSmooth::Calculategradient() {
  gradient_.resize(variables_num_);
  for (uint i = 0; i < points_num_; ++i) {
    Eigen::Vector2d ref_point((double)orin_path_[i].x, (double)orin_path_[i].y);
    gradient_.segment(2 * i, 2) = -2 * deviation_w_ * ref_point;
  }
}



/**
 *@brief:约束线性比例矩阵计算
 */
void FemPosSmooth::CalConstraintMatrix(const vector<vector<float>> &partial_derivative) {
  /*
  *矩阵size:constraints_num_ X variables_num_
  *根据变量X向量size和约束向量size，将矩阵的数值区域分为三块
  *     左上单位矩阵块(vn X vn)->位置约束+slack约束
  *     底部曲率约束矩阵块(ccn X vn)->再次拆分成左右两块矩阵
  *           左侧矩阵(ccn X vn)->F(X_ref)，用于计算F(X_ref)*X_ref
  *           右侧矩阵(ccn X ccn)->负单位矩阵，-slack_i
  */
  /*step01->左上单位矩阵(pos & slack)*/
  Eigen::SparseMatrix<float> p_s_contraint_mat(variables_num_, variables_num_);
  p_s_contraint_mat.setIdentity();
  /*step02->曲率约束矩阵(下部矩阵块)*/
  Eigen::SparseMatrix<float> cc_contraint_mat(curvature_constraints_num_, variables_num_);
  //按行填值
  for (uint i = 0; i < curvature_constraints_num_; ++i) {
    //偏导计算块
    for (uint j = 0; j < 6; ++j) {
      cc_contraint_mat.insert(i, 2 * i + j) = partial_derivative[i][j];
    }
    // slack计算块
    cc_contraint_mat.insert(i, pos_var_num_ + i) = -1.f;
  }
  /*step03->矩阵拼接*/
  Eigen::MatrixXf constraint_mat_dense(constraints_num_, variables_num_);
  constraint_mat_dense.block(0, 0, variables_num_, variables_num_) = p_s_contraint_mat;
  constraint_mat_dense.block(variables_num_, 0, curvature_constraints_num_, variables_num_) = cc_contraint_mat;
  constraint_matrix_ = constraint_mat_dense.sparseView();
}

vector<float> FemPosSmooth::LinearizedOnPos(int index) {
  // X_ref
  float xf = orin_path_[index - 1].x;
  float xm = orin_path_[index].x;
  float xl = orin_path_[index + 1].x;
  float yf = orin_path_[index - 1].y;
  float ym = orin_path_[index].y;
  float yl = orin_path_[index + 1].y;
  // 偏导
  float pd_xf = 2.0 * xf - 4.0 * xm + 2.0 * xl;
  float pd_xm = 8.0 * xm - 4.0 * xf - 4.0 * xl;
  float pd_xl = 2.0 * xl - 4.0 * xm + 2.0 * xf;
  float pd_yf = 2.0 * yf - 4.0 * ym + 2.0 * yl;
  float pd_ym = 8.0 * ym - 4.0 * yf - 4.0 * yl;
  float pd_yl = 2.0 * yl - 4.0 * ym + 2.0 * yf;
  // F(X_ref)
  float F_X_index = (xf + xl - 2 * xm) * (xf + xl - 2 * xm) + (yf + yl - 2 * ym) * (yf + yl - 2 * ym);
  // dF(X_ref)*X_ref
  float dF_m_X_ref = (xf * pd_xf) + (xm * pd_xm) + (xl * pd_xl) + (yf * pd_yf) + (ym * pd_ym) + (yl * pd_yl);
  float linear_approx = F_X_index - dF_m_X_ref;
  return {pd_xf, pd_yf, pd_xm, pd_ym, pd_xl, pd_yl, linear_approx};
}

void FemPosSmooth::CalculateConstraints() {
  vector<vector<float>> lin_cache;
  for (int i = 1; i < points_num_ - 1; ++i) {
    lin_cache.push_back(LinearizedOnPos(i));
  }
  lower_bound_.resize(constraints_num_);
  upper_bound_.resize(constraints_num_);
  //位移约束
  for (uint i = 0; i < points_num_; ++i) {
    Eigen::Vector2d ref_point(orin_path_[i].x, orin_path_[i].y);
    lower_bound_.segment(2 * i, 2) = ref_point - Eigen::Vector2d::Ones() * 0.05;
    upper_bound_.segment(2 * i, 2) = ref_point - Eigen::Vector2d::Ones() * 0.05;
  }
  // slack约束
  for (uint i = pos_var_num_; i < variables_num_; ++i) {
    lower_bound_(i) = 0.0;
    upper_bound_(i) = 1e20;
  }
  //曲率约束
  float constant = powf(delta_s_, 4) * powf(1 / constraint_r_, 2);
  for (int i = 0; i < curvature_constraints_num_; ++i) {
    upper_bound_(variables_num_ + i) = (double)(constant - lin_cache[i][6]);
    lower_bound_(variables_num_ + i) = -1e20;
  }
  //约束线性比例矩阵计算
  CalConstraintMatrix(lin_cache);
}

bool FemPosSmooth::PathSmooth(const vector<port::CommonPose> &path) {
  SetBasicParam(path);
  CalculateHessian();
  Calculategradient();
  CalculateConstraints();
  OsqpEigen::Solver solver;
  solver.settings()->setVerbosity(false);
  solver.settings()->setWarmStart(true);
  solver.data()->setNumberOfVariables(variables_num_);
  solver.data()->setNumberOfConstraints((constraints_num_));
  if (!solver->data()->setHessianMatrix(hessian_matrix_)) return false;
  if (!solver->data()->setGradient(gradient_)) return false;
  if (!solver->data()->setLinearConstraintsMatrix(constraint_matrix_)) return false;
  if (!solver->data()->setLowerBound(lower_bound_)) return false;
  if (!solver->data()->setUpperBound(upper_bound_)) return false;
  // instantiate the solver
  if (!solver->initSolver()) return false;
  if (!solver->solve()) return false;
  smooth_path_.resize(points_num_);
  qp_solution_ = solver.getSolution();
  for (int i = 0; i < points_num_; ++i) {
    smooth_path_[i].x = (float)qp_solution_[2 * i];
    smooth_path_[i].y = (float)qp_solution_[2 * i + 1];
  }
  return ture;
}

}  // namespace planning
}  // namespace modules