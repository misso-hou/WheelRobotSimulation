#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <vector>

#include "common_port/data_port.h"

#define UNCLASSIFIED -1
#define DBSCAN_NOISE -2
#define DBSCAN_SUCCESS 0
#define DBSCAN_FAILURE -3

using namespace std;
namespace port = utilities::port;

namespace utilities {
namespace basicAlg {

class DBSCAN {
 public:
  DBSCAN() {}
  DBSCAN(unsigned int minPts, float eps, vector<port::ClusterPoint> points) {
    m_minPoints = minPts;
    m_epsilon = eps;
    m_points = points;
    m_pointSize = points.size();
  }
  ~DBSCAN() {}

 public:
  pair<vector<port::ClusterPoint>, int> dbscanFit(unsigned int minPts, float eps, vector<Eigen::Vector2f>& points) {
    // 参数设置
    dataSetFunc(minPts, eps, points);
    // 运算
    int clusterID = 0;
    bool index_add_flag = true;
    vector<port::ClusterPoint>::iterator iter;
    for (iter = m_points.begin(); iter != m_points.end(); ++iter) {
      if (iter->clusterID == UNCLASSIFIED) {
        if (expandCluster(*iter, clusterID, index_add_flag) != DBSCAN_FAILURE) {
          index_add_flag = true;
        }
      }
    }
    cluster_num_ = clusterID;
    return make_pair(m_points, cluster_num_);
  }

 private:
  void dataSetFunc(unsigned int minPts, float eps, vector<Eigen::Vector2f>& points) {
    m_minPoints = minPts;
    m_epsilon = eps;
    m_pointSize = points.size();
    m_points.clear();
    for (uint i = 0; i < m_pointSize; i++) {
      port::ClusterPoint local_point;
      local_point.pose = points[i];
      local_point.clusterID = UNCLASSIFIED;
      local_point.index = i;
      m_points.push_back(local_point);
    }
  }

  int expandCluster(port::ClusterPoint point, int& clusterID, bool& add_flag) {
    // 计算核心点附近范围内点
    vector<int> clusterSeeds = calculateCluster(point);
    // 核心点聚类不满足最小点数限制
    if (clusterSeeds.size() < m_minPoints) {
      point.clusterID = DBSCAN_NOISE;
      return DBSCAN_FAILURE;
    } else {
      // 聚类点聚类编号设置
      int index = 0, indexCorePoint = 0;
      vector<int>::iterator iterSeeds;
      for (iterSeeds = clusterSeeds.begin(); iterSeeds != clusterSeeds.end(); ++iterSeeds) {
        if (add_flag) {
          add_flag = false;
          clusterID++;
        }
        m_points.at(*iterSeeds).clusterID = clusterID;
        if (m_points.at(*iterSeeds).pose(0) == point.pose(0) && m_points.at(*iterSeeds).pose(1) == point.pose(1)) {
          indexCorePoint = index;
        }
        ++index;
      }
      clusterSeeds.erase(clusterSeeds.begin() + indexCorePoint);

      for (vector<int>::size_type i = 0, n = clusterSeeds.size(); i < n; ++i) {
        vector<int> clusterNeighors = calculateCluster(m_points.at(clusterSeeds[i]));

        if (clusterNeighors.size() >= m_minPoints) {
          vector<int>::iterator iterNeighors;
          for (iterNeighors = clusterNeighors.begin(); iterNeighors != clusterNeighors.end(); ++iterNeighors) {
            if (m_points.at(*iterNeighors).clusterID == UNCLASSIFIED || m_points.at(*iterNeighors).clusterID == DBSCAN_NOISE) {
              if (m_points.at(*iterNeighors).clusterID == UNCLASSIFIED) {
                clusterSeeds.push_back(*iterNeighors);
                n = clusterSeeds.size();
              }
              m_points.at(*iterNeighors).clusterID = clusterID;
            }
          }
        }
      }

      return DBSCAN_SUCCESS;
    }
  }

  vector<int> calculateCluster(port::ClusterPoint point) {
    int index = 0;
    vector<port::ClusterPoint>::iterator iter;
    vector<int> clusterIndex;
    for (iter = m_points.begin(); iter != m_points.end(); ++iter) {
      if (calculateDistance(point, *iter) <= m_epsilon) {
        clusterIndex.push_back(index);
      }
      index++;
    }
    return clusterIndex;
  }

  inline float calculateDistance(const port::ClusterPoint& pointCore, const port::ClusterPoint& pointTarget) {
    return pow(pointCore.pose(0) - pointTarget.pose(0), 2) + pow(pointCore.pose(1) - pointTarget.pose(1), 2);
  }

  pair<vector<port::ClusterPoint>, int> GetResult() { return make_pair(m_points, cluster_num_); }

  int getTotalPointSize() { return m_pointSize; }

  int getMinimumClusterSize() { return m_minPoints; }

  int getEpsilonSize() { return m_epsilon; }

 public:
  vector<port::ClusterPoint> m_points;

 private:
  unsigned int m_pointSize;
  unsigned int m_minPoints;
  float m_epsilon;
  int cluster_num_;  //聚类个数（与cluster ID对应)
};

}  // namespace basicAlg
}  // namespace utilities