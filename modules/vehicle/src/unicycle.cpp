#include "unicycle.h"

namespace modules {
namespace vehicle {

port::CommonPose Unicycle::StateUpdate(const port::CommonPose& cur_pose, const float& cur_v, const float& cur_w, const float& dt) {
  port::CommonPose next_pose;
  next_pose.x = cur_pose.x + cur_v * cos(cur_pose.theta) * dt;
  next_pose.y = cur_pose.y + cur_v * sin(cur_pose.theta) * dt;
  float angle = cur_pose.theta + cur_w * dt;
  next_pose.theta = atan2(sin(angle), cos(angle));
  return next_pose;
}

tuple<smatf, smatf, smatf> Unicycle::TrackingErrorSS(const float& v_ref, const float& K, const float& dt) {
  /* 非线性跟踪误差模型(只考虑横向误差)
   * {state}:[lte,theta_e] ; {input}:[w]
   * lte=v_r*sin(theta_e)~=v_r*theta_e                   (小角度定理)
   * theta_e=w_r-w=v_r*K*cos(theta_e)-w~=v_r*K-w         (小角度定理)
   * 离散化:在x=[0,0],u=[w_r]处展开
   * lte_dot~=v_r*theta_e
   * theta_e_dot~=v_r*K-w_r-(w-w_r)=v_r*K-w
   */
  Eigen::SparseMatrix<float> A(2, 2);
  A.insert(0, 1) = v_ref;
  Eigen::SparseMatrix<float> B(2, 1);
  B.insert(1, 0) = -1;
  Eigen::SparseMatrix<float> W(2, 1);
  W.insert(1, 0) = v_ref * K;
  // 离散化处理
  Eigen::SparseMatrix<float> I(2, 2);
  I.setIdentity();
  A = I + A * dt;  // 欧拉前向差分
  B = B * dt;
  W = W * dt;
  return make_tuple(A, B, W);
}

int Unicycle::GetStateNum() { return n_state_; }

int Unicycle::GetInputNum() { return n_input_; }

pair<Eigen::VectorXd, Eigen::VectorXd> Unicycle::GetInputLimit() {
  Eigen::VectorXd min_input(n_input_), max_input(n_input_);
  min_input[0] = min_w_;
  max_input[0] = max_w_;
  return make_pair(min_input, max_input);
}

pair<Eigen::VectorXd, Eigen::VectorXd> Unicycle::GetInputDotLimit() {
  Eigen::VectorXd min_input_dot(n_input_), max_input_dot(n_input_);
  min_input_dot[0] = min_w_dot_;
  max_input_dot[0] = max_w_dot_;
  return make_pair(min_input_dot, max_input_dot);
}

}  // namespace vehicle
}  // namespace modules