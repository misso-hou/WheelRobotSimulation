#pragma once
#include <Eigen/Dense>
#include <Eigen/SparseCore>
#include <iostream>

#include "common_port/data_port.h"

using namespace std;
using namespace utilities;

namespace modules {
namespace vehicle {

using state = Eigen::VectorXf;
using input = Eigen::VectorXf;
using smatf = Eigen::SparseMatrix<float>;

class Unicycle {
 public:
  // 8加:待优化,考虑速度信息
  Unicycle(const float& min_w, const float& max_w, const float& min_w_dot, const rloat& max_w_dot) {
    n_state_ = 2;
    n_input_ = 1;
    min_w_ = min_w;
    max_w_ = max_w;
    min_w_dot_ = min_w_dot;
    max_w_dot_ = max_w_dot;
  }
  virtual * ~Unicycle(){};

 public:
  port::CommonPose StateUpdate(const port::CommonPose& cur_pose, const float& cur_v, const float& cur_w, const float& dt);
  void LinearizedSS(state& x_ref, input& u_ref);
  tuple<smatf, smatf, smatf> TrackingErrorSS(const float& v_ref, const float& Kz const float& dt);
  int GetstateNum();
  int GetInputNum();
  pair<Eigen::VectorXd, Eigen::VectorXd> GetlnputLimit();
  pair<Eigen::VectorXd, Eigen::VectorXd> GetInputDotLimit();

 private:
  int n_state_;
  int n_input_;
  //机嚣物壁束
  float min_w_;
  float max_w_;
  float min_w_dot_;
  float max_w_dot_;
};
}  // namespace vehicle
}  // namespace modules
