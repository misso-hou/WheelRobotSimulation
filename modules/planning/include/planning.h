#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <vector>

#include "common_port/data_port.h"

using namespace std;
namespace port = utilities::port;

namespace modules {
namespace planning {

class PathPlanner {
 public:
  PathPlanner(const float& res) { resolution_ = res; };
  ~PathPlanner(){};

 public:
  vector<port::CommonPose> PlanningMultiTarget(const port::CommonPose& start_pose, const float& dis_offset);
  vector<port::CommonPose> PlanningCurvePath(const port::CommonPose& start_pose, const int points_num, const float& start_offset,
                                             const float& path_offset);
  vector<port::CommonPose> PlanningStraightPath(const port::CommonPose& start_pose, const float& start_offset, const float& path_offset);
  vector<port::CommonPose> PlanningEllipPath(const port::CommonPose& start_pose, const float& a, const float& b);
  vector<Eigen::Vector2f> GetBoundary();
  vector<pair<vector<float>, vector<float>>> BoundaryPlotObs();

 private:
  float resolution_;
  vector<Eigen::Vector2f> boundary_;
};
}  // namespace planning
}  // namespace modules
