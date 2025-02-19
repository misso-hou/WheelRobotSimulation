#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <vector>

#include "common_port/data_port.h"

namespace modules {
namespace sensor {

using namespace std;
namespace port = utilities::port;

class LidarSensor {
 public:
  LidarSensor(float res, float range);
  ~LidarSensor(){};
  vector<Eigen::Vector2f> UpdateAiSensor(const port::CommonPose& robot_pose, const vector<pair<vector<float>, vector<float>>>& env_obs);

  vector<vector<float>> GetLidarData();

 private:
  vector<vector<float>> ObsDetectByReflect(const port::CommonPose& sensor_pose, const vector<pair<vector<float>, vector<float>>>& env_obs);

  vector<vector<float>> ObsDetectByAngle(const port::CommonPose& sensor_pose, const vector<pair<vector<float>, vector<float>>>& env_obs);

 private:
  float range_;
  float resolution_;
  int sector_num_;
  vector<vector<float>> lidar_data_;
};

}  // namespace sensor
}  // namespace modules
