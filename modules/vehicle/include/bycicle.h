#pragma once

#include <Eigen/Dense>
#include <Eigen/SparseCore>
#include <iostream>

#include "common_port/data_port.h"

using namespace std;

namespace modules {
namespace vehicle {

using state = Eigen::VectorXf;
using input = Eigen::VectorXf;
using matXf = Eigen::MatrixXf;

class Bycicle {
 public:
  Bycicle(const float& wheel_base) {
    wheel_base_ = wheel_base;
    int n_state_ = 3;
    int n_input_ = 1;
  }
  virtual ~Bycicle(){};

  state StateUpdate(const state& x, const input& u, const float& dt);
  void LinearizedSS(state& x_ref, input& u_ref);
  tuple<matXf, matXf, matXf> TrackingErrorSS(const float& v_ref, const float& K, const float& dt);
  void DiscreteRK4(matXf& A, matXf& B, state& X, input& U, float& sample_period);
  int GetStateNum();
  int GetInputNum();

 private:
  float wheel_base_;
  int n_state_;
  int n_input_;
  int RK_steps_;
};

}  // namespace vehicle
}  // namespace modules
