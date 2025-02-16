#include "bycicle.h"

namespace modules {
namespace vehicle {

state Bycicle::StateUpdate(const state& x, const input& u, const float& dt) {
  state x_dot(4), next_x(4);
  x_dot[0] = x[3] * cos(x[2]);
  x_dot[1] = x[3] * sin(x[2]);
  x_dot[2] = x[3] * tan(u[0] / wheel_base_);
  x_dot[3] = u[1];
  next_x = x + x_dot * dt;
  return next_x;
}

tuple<matXf, matXf, matXf> Bycicle::TrackingErrorSS(const float& v_ref, const float& K, const float& dt) {
  /* 非线性跟踪误差模型(只考虑横向误差)
   * {state}:[lte,theta_e,delta_e] ; {input}:[delta]
   * lte=v_r*sin(theta_e)~=v_r*theta_e
   * theta_e=w_r-w=v_r*K*cos(theta_e)-v_r*tan(delta)/L~=v_r*K-v_r*tan(delta)/L
   * 离散化->在x=[lte=0,theta_e=0,delta_e=0];u=[delta_r]处展开
   * lte_dot~=v_r(theta_e-theta_er)=v_r*theta_e
   * theta_e_dot~=v_r*K-v_r*tan(delta_r)/L-(v_r/L)*d(tan(delta))/d(delta)*(delta-delta_r)
   *             =v_r*K-v_r*tan(delta_r)/L-(v_r/L)*1/cos(delta_r)**2*(delta-delta_r)   note:delta_r=arctan(L*K)
   *             =v_r/L*(delta-delta_r)/cos(delta_r)**2      
   */
  matXf A = matXf::Zero(2, 2);
  A(0, 1) = v_ref;
  matXf B = matXf::Zero(2, 1);
  float delta_r = atan(wheel_base_ * K);
  B(1, 0) = v_ref / (wheel_base_ * powf(cos(delta_r), 2));
  matXf W = matXf::Zero(2, 1);
  W(1, 0) = -v_ref * delta_r / (wheel_base_ * powf(cos(delta_r), 2));
  // 离散化处理
  matXf I = matXf::Identity(2, 2);
  A = I + A * dt;  // 欧拉前向差分
  B = B * dt;
  W = W * dt;
  return make_tuple(A, B, W);
}

int Bycicle::GetStateNum() { return n_state_; }

int Bycicle::GetInputNum() { return n_input_; }

}  // namespace vehicle
}  // namespace modules