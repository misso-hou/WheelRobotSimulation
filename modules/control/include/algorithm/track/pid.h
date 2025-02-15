#pragma once

#include <cmath>
#include <iostream>

namespace modules {
namespace control {
namespace algorithm {

// PID Controller //
class PID {
 private:
  float Kp_, Ki_, Kd_;
  float curError_, prevError_, integral_, derivative_;
  float max_integral_, max_output_, min_integral_, min_output_;

 public:
  PID(float MAX_output = 1000, float MAX_integral = 1000) {
    /*抗积分包和参数*/
    max_integral_ = MAX_integral;
    min_integral_ = -MAX_integral;
    /*控制输出限制*/
    max_output_ = MAX_output;
    min_output_ = -MAX_output;
    /*初始化误差*/
    prevError_ = 0.0;
    integral_ = 0.0;
    derivative_ = 0.0;
  }
  ~PID(){};

 public:
  void SetCoreParam(float kP, float kD = 0.0f, float kI = 0.0f);
  void ControllerReset(void);
  float Compute(float error);
  void ClearIntergral(void);
};

}  // namespace algorithm
}  // namespace control
}  // namespace modules