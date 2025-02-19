#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <shared_mutex>

#include "common_port/data_port.h"
#include "facilities/singleton.h"

namespace modules {
namespace datacenter {

using namespace std;
namespace port = utilities::port;

using poses = vector<port::CommonPose>;
using mesh2D = vector<vector<float>>;

class DataCenter : public utilities::Singleton<DataCenter> {
  friend class Singleton<DataCenter>;

 private:
  DataCenter(){};
  ~DataCenter(){};

  //数据接收函数
 public:
  void SetRobotPose(const port::CommonPose& pose);
  void SetAiSyncRobotPose(const port::CommonPose& pose);
  void SetTargetPose(const port::CommonPose& pose);
  void SetTestTempPose(const port::CommonPose& pose);
  void SetNearestPointId(const int id);
  void SetPlanningPath(const poses& path);
  void SetSpCurves(const poses& sp_curves);
  void SetDgBorders(const poses& dg_borders);
  void SetTrajectoryPath(const vector<port::TrajPoint>& path);
  void SetMpcHorizon(const poses& mpc_horizon);
  void SetCmdVel(const port::Twist& cmd);
  void SetPlotObsPoints(const vector<pair<vector<float>, vector<float>>>& points);
  void SetObsPointsCloud(const vector<Eigen::Vector2f>& obs_points);
  void SetBoundary(const vector<Eigen::Vector2f>& boundary);
  void SetAiObs(const vector<Eigen::Vector2f>& ai_obs);
  void SetMapObs(const vector<Eigen::Vector2f>& map_obs);
  void SetLidarData(const vector<vector<float>>& data);
  void SetCsvSwitch(const bool flag);
  void SetPathUpdateFlag(const bool flag);
  //动态系统避障
  void SetDsAvoidAgents(const vector<port::CircleAgent>& agents);
  void SetAvoidSamplePoints(const vector<port::SamplePoint>& s_points);
  void SetAvoidClusterGroup(const vector<vector<port::ClusterPoint>>& groups);
  void SetAvoidVirtualObsCenter(const vector<Eigen::Vector2f>& obs_centers);
  void SetAvoidObsNearestPoints(const vector<Eigen::Vector2f>& points);
  void SetAvoidVirtualObs(const vector<port::VirtualObs>& avoid_obs);
  void SetAvoidPlanningVel(const Eigen::Vector3f& avoid_agent_vel);

  //数据发送函数
 public:
  port::CommonPose GetRobotPose();
  port::CommonPose GetAiSyncRobotPose();
  port::CommonPose GetTargetPose();
  port::CommonPose GetTestTempPose();
  int GetNearestPointId();
  poses GetPlanningPath();
  poses GetSpCurves();
  poses GetDgBorders();
  vector<port::TrajPoint> GetTrajectoryPath();
  poses GetMpcHorizon();
  port::Twist GetCmdVel();
  vector<pair<vector<float>, vector<float>>> GetPlotObsPoints();
  vector<Eigen::Vector2f> GetObsPointsCloud();
  vector<Eigen::Vector2f> GetBoundary();
  vector<Eigen::Vector2f> GetAiObs();
  vector<Eigen::Vector2f> GetMapObs();
  vector<vector<float>> GetLidarData();
  bool GetCsvSwitch();
  bool GetPathUpdateFlag();
  //动态系统避障
  vector<port::CircleAgent> GetDsAvoidAgents();
  vector<port::SamplePoint> GetAvoidSamplePoints();
  vector<vector<port::ClusterPoint>> GetAvoidClusterGroup();
  vector<Eigen::Vector2f> GetAvoidVirtualObsCenter();
  vector<Eigen::Vector2f> GetAvoidObsNearestPoints();
  vector<port::VirtualObs> GetAvoidVirtualObs();
  Eigen::Vector3f GetAvoidPlanningVel();

  //仿真可视化数据
 public:
  mesh2D GetPltTrackingPath();
  mesh2D GetPltSharpCurves();
  mesh2D GetPltDangerousBorder();
  mesh2D GetPltTrajectroy(const int buffer_length, const bool erase = false);
  mesh2D GetRobotOutline(const port::CommonPose& robot_pose);
  mesh2D GetRobotDriveWheel(const port::CommonPose& robot_pose, int side);
  mesh2D GetRobotOmniWheel(const port::CommonPose& robot_pose, int side, const float& steer_angle);
  vector<mesh2D> GetVisionBoundary(const port::CommonPose& robot_pose);
  mesh2D GetPltBoundary();

 private:
  shared_mutex data_mutex_;  //数据传输锁
  //点信息
  port::CommonPose robot_pose_;
  port::CommonPose ai_sync_robot_pose_;
  port::CommonPose target_pose_;
  port::CommonPose test_temp_pose_;
  int nearest_point_id_;
  //路径信息
  poses planning_path_;
  poses plt_sharp_curves_;
  poses plt_dg_borders_;
  vector<port::TrajPoint> traj_path_;
  poses mpc_horizon_;
  //控制相关信息
  port::Twist cmd_vel_;
  //障碍物相关信息
  vector<pair<vector<float>, vector<float>>> plot_obs_points_;
  vector<Eigen::Vector2f> obs_points_cloud_;
  vector<Eigen::Vector2f> ai_obs_;
  vector<Eigen::Vector2f> map_obs_;
  vector<vector<float>> lidar_data_;
  vector<Eigen::Vector2f> boundary_;
  //状态标志
  bool csv_logger_switch_;
  bool path_update_flag_;
  //动态系统避障
  vector<port::CircleAgent> avoid_agents_;
  vector<port::SamplePoint> avoid_sample_points_;
  vector<vector<port::ClusterPoint>> avoid_cluster_groups_;
  vector<Eigen::Vector2f> avoid_vritual_obs_center_;
  vector<Eigen::Vector2f> avoid_obs_nearest_points_;
  vector<port::VirtualObs> avoid_virtual_obs_;
  Eigen::Vector3f avoid_planning_vel_;
};
}  // namespace datacenter
}  // namespace modules
