#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <utility>
#include <vector>

#include "common_port/data_port.h"

namespace modules {
namespace env {

using namespace std;
namespace port = utilities::port;

typedef enum { ELLIP, RANDOM, Rectangle } ObsType;

typedef enum { COUNTOUR, COUNTOURCLOUD } DataType;

class Obstacle {
 public:
  Obstacle() {}
  Obstacle(ObsType type, const port::CommonPose& pose, const port::Twist& vel, float dt);
  ~Obstacle() {}

 private:  //用于计算的变量
  port::CommonPose center_pose_;
  port::Twist obs_vel_;
  float dt_;
  //障碍物类型
  ObsType obs_type_;
  //障碍物数据类型（面点&轮廓点）
  DataType data_type_;
  Eigen::MatrixXf obs_points_;
  Eigen::MatrixXf origin_obs_points_;
  int points_num_;
  vector<float> random_r_;

 private:
  //用于动画的变量
  pair<vector<float>, vector<float>> obs_show_;
  //传输计算的变量
  vector<Eigen::Vector2f> trans_obs_points_;

  //障碍物参数
 private:
  float sdev_;
  //椭圆障碍物参数
  Eigen::Array2f axis_length_;
  //放射形障碍物参数
  float radius_;
  int edge_num_;
  //矩形障碍物i参数
  float height_;
  float width_;
  float ds_;

 private:  //内部计算函数
  void GenerateEllisoidObs();
  void GenerateRandomShapeObs();
  void GenerateRectangleVertexObs();
  void ObsTransform();
  void UpdateObsState();
  void CloudObsUpdate();
  void CoutourObsUpdate();
  void ContourInitialize();
  void CloudRandomObsInit();
  void EllipCloudObsUpdate();
  void RectCloudObsUpdate();
  void RandomCloudObsUpdate();

 public:
  void ObsInit();
  void ObsUpdate();
  void SetEllipsoidAxis(const Eigen::Array2f& axis_length, int points_num, const float& ds = 0.0f);
  void SetRandomObsParam(const float& radius, int edge_num, int points_num);
  void SetRectangleObsParam(const float& height, const float& width, const float& ds);
  void SetObsProperty(ObsType type, DataType dtype, const port::CommonPose& pose, const port::Twist& vel, const float& dt, const float& dev = 0.0f);
  pair<vector<float>, vector<float>> GetObsToPlot() { return obs_show_; }
  vector<Eigen::Vector2f> GetUpdatedObs() { return trans_obs_points_; }
};

}  // namespace env
}  // namespace modules