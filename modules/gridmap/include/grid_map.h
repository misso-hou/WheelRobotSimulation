#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <vector>

#include "common_port/data_port.h"



namespace modules {
namespace gridmap {

// using namespace std;
namespace port = utilities::port;

//地图内部数据类型定义
using Matrix = Eigen::MatrixXf;
using DataType = Matrix::Scalar;
using Position = Eigen::Vector2f;
using Vector = Eigen::Vector2f;
using Index = Eigen::Array2i;
using Size = Eigen::Array2i;
using Length = Eigen::Array2f;
using Time = uint64_t;

class GridMap {
 public:
  GridMap(const float& height, const float& width, const float& resolution, const float& back_height);
  virtual ~GridMap() = default;
  vector<Position> GridMapUpdate(const vector<Position>& sensor_data, const port::CommonPose& sync_pose);

 private:
  void SensorEraseLayerInit();
  bool IsInPolygon(const Position& pos, const vector<Position>& polygon);

 private:
  Matrix map_data_;
  Matrix sensor_erase_data_;
  Length length_;
  float back_height_;
  float resolution_;
  Size size_;
  Size offset_;
  vector<Position> global_orin_obs_;
  vector<Position> local_orin_obs_;
  bool sensor_polygon_init_;
  vector<vector<Position>> sensor_polygon_;
};

}  // namespace gridmap
}  // namespace modules