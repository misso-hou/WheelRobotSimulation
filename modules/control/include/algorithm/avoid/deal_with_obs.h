#pragma once

#include <iostream>

#include "avoid_base.h"

#include "basic_algorithm/dbscan.hpp"
#include "basic_algorithm/interpolation.hpp"

namespace modules {
namespace control {
namespace algorithm {

//简化命名空间
namespace port = utilities::port;
namespace basicAlg = utilities::basicAlg;
//变量名简化
using vec2f = Eigen::Vector2f;
using vvec2f = vector<vec2f>;
using vvcp = vector<vector<port::ClusterPoint>>;

class DealWithObs {
 public:
  DealWithObs(int pmn, const float& eps, const float& idis);
  ~DealWithObs() {}

 private:
  basicAlg::DBSCAN dbscan_;
  basicAlg::InterpolateTools itp_tools_;

 private:
  int points_min_num_;   // minimum number of cluster
  int epsilon_;          // distance for clustering, metre^2
  int interpolate_dis_;  // 样条插值距离 //todo:疏松->影响切线稳定性，稠密->影响计算效率，寻找更高效方式
 private:
  vvec2f ExtractInBoundaryObs(const vvec2f& obs_points);
  int CalSampledPoints(const vvec2f& obs_points);
  vvcp SamplePointsCluster(int min_pt_num, const float& dis_thd);
  vvcp ObsPointsConnect(const vvcp& cluster_group, const float& connect_thd);
  vector<port::VirtualObs> ObsPointsInterpolation(const vvcp& cluster_obs);
  int CalVirtualITPLObs(const float& connect_thd);
  vvcp ConvexClusterGroup(const vvcp& cluster_group);
  int CalVirtualVertexObs();
  pair<vec2f, float> CalPlygonObsNormal(const vec2f& focus, const port::VirtualObs& obstacle);
  pair<vec2f, float> CalPointsCloudNormal(const vec2f& focus, const port::VirtualObs& obstacle);
  pair<vec2f, float> CalObsNormalVec(const vec2f& pose, const port::VirtualObs& v_obs);
  vector<port::CalculatedObs> ComputeAgentsCalObs(const port::CircleAgent& agent);

 public:
  int DisposeObs(const vvec2f& obs_points);
};

}  // namespace algorithm
}  // namespace control
}  // namespace modules