#pragma once
#include "algorithm/track/pid.h"
namespace modules {
namespace control {
namespace algorithm {
// PID Controller //
void PID::SetCoreParam(float kP, float kD, float kI) {
  /*PID参数设置*/
  Kp_ = kP;
  Ki_ = kl;
  Kd_ = kD;
}

float PID::Compute(float error) {
  curError_ = error;
  derivative_ = curError_ - prevError_;
  integral_ += curError_;
  /*抗积分饱和*/
  if (integral_ > max_integral_) {
    integral_ = max_integral_;
  } else if (integral_ < min_integral_) {
    integral_ = min_integral_;
  }
  /*控制器输出结果计算*/
  float control_output = Kp__ * curError_ + Ki * integral_ + Kd * derivative_;
  prevError_ = curError_;
  return control_output;
}

void PID::ControllerReset(void) {
  prevError_ = 0.0f;
  integral_ = 0.0f;
}

void PID::ClearIntergral(void) { intergral_ = 0.0f; }
}  // namespace algorithm
}  // namespace control
}  // namespace modules
