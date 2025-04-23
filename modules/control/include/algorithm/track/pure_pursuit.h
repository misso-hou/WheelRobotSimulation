#pragma once

#include <iostream>

#include "common_port/data_port.h"

namespace modules {
namespace control {
namespace algorithm {

using namespace std;
namespace port = utilities::port;

class PurePursuit {
 public:
  PurePursuit(float max_v, float lD, float min_ld) : max_v_(max_v), ld_(lD), min_ld_(min_ld) {}
  int GetTargetIndex(const port::CommonPose& pose, const float& linear_v, const vector<port::CommonPose>& path, const int index);

  float PurePursuitControl(const port::CommonPose& pose, const float& linear_v, const vector<port::CommonPose>& points, int& targetIdx);

 private:
  float ld_;      //最大预瞄距离
  float max_v_;   //比例系数(暂时未使用)
  float min_ld_;  //最小预瞄准距离
  float angAlpha_;
};

}  // namespace algorithm
}  // namespace control
}  // namespace modules
