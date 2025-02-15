#include "data_center.h"

#include "robot_configuration.h"

namespace modules {
namespace datacenter {

namespace robot = modules::vehicle;

void DataCenter::SetRobotPose(const port::CommonPose& pose) {
  std::unique_lock<std::shared_mutex> lock(data_mutex_);
  robot_pose_ = pose;
}

void DataCenter::SetAiSyncRobotPose(const port::CommonPose& pose) {
  std::unique_lock<std::shared_mutex> lock(data_mutex_);
  ai_sync_robot_pose_ = pose;
}

void DataCenter::SetTargetPose(const port::CommonPose& pose) {
  std::unique_lock<std::shared_mutex> lock(data_mutex_);
  target_pose_ = pose;
}

void DataCenter::SetTestTempPose(const port::CommonPose& pose) {
  std::unique_lock<std::shared_mutex> lock(data_mutex_);
  test_temp_pose_ = pose;
}

void DataCenter::SetNearestPointId(const int id) {
  std::unique_lock<std::shared_mutex> lock(data_mutex_);
  nearest_point_id_ = id;
}

void DataCenter::SetPlanningPath(const poses& path) {
  std::unique_lock<std::shared_mutex> lock(data_mutex_);
  planning_path_ = path;
}

void DataCenter::SetSpCurves(const poses& sp_curves) {
  std::unique_lock<std::shared_mutex> lock(data_mutex_);
  plt_sharp_curves_ = sp_curves;
}

void DataCenter::SetDgBorders(const poses& dg_borders) {
  std::unique_lock<std::shared_mutex> lock(data_mutex_);
  plt_dg_borders_ = dg_borders;
}

void DataCenter::SetTrajectoryPath(const vector<port::TrajPoint>& path) {
  std::unique_lock<std::shared_mutex> lock(data_mutex_);
  traj_path_ = path;
}

void DataCenter::SetMpcHorizon(const poses& mpc_horizon) {
  std::unique_lock<std::shared_mutex> lock(data_mutex_);
  mpc_horizon_ = mpc_horizon;
}

void DataCenter::SetCmdVel(const port::Twist& cmd) {
  std::unique_lock<std::shared_mutex> lock(data_mutex_);
  cmd_vel_ = cmd;
}

void DataCenter::SetPlotObsPoints(const vector<pair<vector<float>, vector<float>>>& points) {
  std::unique_lock<std::shared_mutex> lock(data_mutex_);
  plot_obs_points_ = points;
}

void DataCenter::SetObsPointsCloud(const vector<Eigen::Vector2f>& obs_points) {
  std::unique_lock<std::shared_mutex> lock(data_mutex_);
  obs_points_cloud_ = obs_points;
}

void DataCenter::SetBoundary(const vector<Eigen::Vector2f>& boundary) {
  std::unique_lock<std::shared_mutex> lock(data_mutex_);
  boundary_ = boundary;
}

void DataCenter::SetAiObs(const vector<Eigen::Vector2f>& ai_obs) {
  std::unique_lock<std::shared_mutex> lock(data_mutex_);
  ai_obs_ = ai_obs;
}

void DataCenter::SetMapObs(const vector<Eigen::Vector2f>& map_obs) {
  std::unique_lock<std::shared_mutex> lock(data_mutex_);
  map_obs_ = map_obs;
}

void DataCenter::SetLidarData(const vector<vector<float>>& data) {
  std::unique_lock<std::shared_mutex> lock(data_mutex_);
  lidar_data_ = data;
}

void DataCenter::SetCsvSwitch(const bool flag) {
  std::unique_lock<std::shared_mutex> lock(data_mutex_);
  csv_logger_switch_ = flag;
}

void DataCenter::SetPathUpdateFlag(const bool flag) {
  std::unique_lock<std::shared_mutex> lock(data_mutex_);
  path_update_flag_ = flag;
}

//动态系统避障
void DataCenter::SetDsAvoidAgents(const vector<port::CircleAgent>& agents) {
  std::unique_lock<std::shared_mutex> lock(data_mutex_);
  avoid_agents_ = agents;
}

void DataCenter::SetAvoidSamplePoints(const vector<port::SamplePoint>& s_points) {
  std::unique_lock<std::shared_mutex> lock(data_mutex_);
  avoid_sample_points_ = s_points;
}

void DataCenter::SetAvoidClusterGroup(const vector<vector<port::ClusterPoint>>& groups) {
  std::unique_lock<std::shared_mutex> lock(data_mutex_);
  avoid_cluster_groups_ = groups;
}

void DataCenter::SetAvoidVirtualObsCenter(const vector<Eigen::Vector2f>& obs_centers) {
  std::unique_lock<std::shared_mutex> lock(data_mutex_);
  avoid_vritual_obs_center_ = obs_centers;
}

void DataCenter::SetAvoidObsNearestPoints(const vector<Eigen::Vector2f>& points) {
  std::unique_lock<std::shared_mutex> lock(data_mutex_);
  avoid_obs_nearest_points_ = points;
}

void DataCenter::SetAvoidVirtualObs(const vector<port::VirtualObs>& avoid_obs) {
  std::unique_lock<std::shared_mutex> lock(data_mutex_);
  avoid_virtual_obs_ = avoid_obs;
}

void DataCenter::SetAvoidPlanningVel(const Eigen::Vector3f& avoid_agent_vel) {
  std::unique_lock<std::shared_mutex> lock(data_mutex_);
  avoid_planning_vel_ = avoid_agent_vel;
}

/**********************************************************/

port::CommonPose DataCenter::GetRobotPose() {
  std::shared_lock<std::shared_mutex> lock(data_mutex_);
  return robot_pose_;
}
port::CommonPose DataCenter::GetAiSyncRobotPose() {
  std::shared_lock<std::shared_mutex> lock(data_mutex_);
  return ai_sync_robot_pose_;
}
port::CommonPose DataCenter::GetTargetPose() {
  std::shared_lock<std::shared_mutex> lock(data_mutex_);
  return target_pose_;
}
int DataCenter::GetNearestPointId() {
  std::shared_lock<std::shared_mutex> lock(data_mutex_);
  return nearest_point_id_;
}
poses DataCenter::GetPlanningPath() {
  std::shared_lock<std::shared_mutex> lock(data_mutex_);
  return planning_path_;
}
poses DataCenter::GetSpCurves() {
  std::shared_lock<std::shared_mutex> lock(data_mutex_);
  return plt_sharp_curves_;
}
poses DataCenter::GetDgBorders() {
  std::shared_lock<std::shared_mutex> lock(data_mutex_);
  return plt_dg_borders_;
}
vector<port::TrajPoint> DataCenter::GetTrajectoryPath() {
  std::shared_lock<std::shared_mutex> lock(data_mutex_);
  return traj_path_;
}
poses DataCenter::GetMpcHorizon() {
  std::shared_lock<std::shared_mutex> lock(data_mutex_);
  return mpc_horizon_;
}
port::Twist DataCenter::GetCmdVel() {
  std::shared_lock<std::shared_mutex> lock(data_mutex_);
  return cmd_vel_;
}
vector<pair<vector<float>, vector<float>>> DataCenter::GetPlotObsPoints() {
  std::shared_lock<std::shared_mutex> lock(data_mutex_);
  return plot_obs_points_;
}
vector<Eigen::Vector2f> DataCenter::GetObsPointsCloud() {
  std::shared_lock<std::shared_mutex> lock(data_mutex_);
  return obs_points_cloud_;
}
vector<Eigen::Vector2f> DataCenter::GetBoundary() {
  std::shared_lock<std::shared_mutex> lock(data_mutex_);
  return boundary_;
}
vector<Eigen::Vector2f> DataCenter::GetAiObs() {
  std::shared_lock<std::shared_mutex> lock(data_mutex_);
  return ai_obs_;
}
vector<Eigen::Vector2f> DataCenter::GetMapObs() {
  std::shared_lock<std::shared_mutex> lock(data_mutex_);
  return map_obs_;
}
vector<vector<float>> DataCenter::GetLidarData() {
  std::shared_lock<std::shared_mutex> lock(data_mutex_);
  return lidar_data_;
}
bool DataCenter::GetCsvSwitch() {
  std::shared_lock<std::shared_mutex> lock(data_mutex_);
  return csv_logger_switch_;
}
bool DataCenter::GetPathUpdateFlag() {
  std::shared_lock<std::shared_mutex> lock(data_mutex_);
  return path_update_flag_;
}

mesh2D DataCenter::GetPltTrackingPath() {
  std::shared_lock<std::shared_mutex> lock(data_mutex_);
  //路径数据结构转换
  vector<float> x_array, y_array;
  for (auto point : planning_path_) {
    x_array.push_back(point.x);
    y_array.push_back(point.y);
  }
  return {x_array, y_array};
}

mesh2D DataCenter::GetPltSharpCurves() {
  std::shared_lock<std::shared_mutex> lock(data_mutex_);
  vector<float> curve_x_array, curve_y_array;
  for (auto point : plt_sharp_curves_) {
    curve_x_array.push_back(point.x);
    curve_y_array.push_back(point.y);
  }
  return {curve_x_array, curve_y_array};
}

mesh2D DataCenter::GetPltBoundary() {
  std::shared_lock<std::shared_mutex> lock(data_mutex_);
  vector<float> curve_x_array, curve_y_array;
  for (auto point : boundary_) {
    curve_x_array.push_back(point(0));
    curve_y_array.push_back(point(1));
  }
  return {curve_x_array, curve_y_array};
}

mesh2D DataCenter::GetPltDangerousBorder() {
  std::shared_lock<std::shared_mutex> lock(data_mutex_);
  vector<float> dg_x_array, dg_y_array;
  for (auto point : plt_dg_borders_) {
    dg_x_array.push_back(point.x);
    dg_y_array.push_back(point.y);
  }
  return {dg_x_array, dg_y_array};
}

mesh2D DataCenter::GetPltTrajectroy(const int buffer_length, const bool erase) {
  std::shared_lock<std::shared_mutex> lock(data_mutex_);
  static vector<float> x_array, y_array;
  if (erase) {
    x_array.clear();
    y_array.clear();
  }
  if (x_array.size() < buffer_length) {
    x_array.push_back(robot_pose_.x);
    y_array.push_back(robot_pose_.y);
  } else {
    x_array.erase(x_array.begin());
    y_array.erase(y_array.begin());
  }
  return {x_array, y_array};
}

//动态系统避障
vector<port::CircleAgent> DataCenter::GetDsAvoidAgents() {
  std::shared_lock<std::shared_mutex> lock(data_mutex_);
  return avoid_agents_;
}
vector<port::SamplePoint> DataCenter::GetAvoidSamplePoints() {
  std::shared_lock<std::shared_mutex> lock(data_mutex_);
  return avoid_sample_points_;
}
vector<vector<port::ClusterPoint>> DataCenter::GetAvoidClusterGroup() {
  std::shared_lock<std::shared_mutex> lock(data_mutex_);
  return avoid_cluster_groups_;
}
vector<Eigen::Vector2f> DataCenter::GetAvoidVirtualObsCenter() {
  std::shared_lock<std::shared_mutex> lock(data_mutex_);
  return avoid_vritual_obs_center_;
}
vector<Eigen::Vector2f> DataCenter::GetAvoidObsNearestPoints() {
  std::shared_lock<std::shared_mutex> lock(data_mutex_);
  return avoid_obs_nearest_points_;
}
vector<port::VirtualObs> DataCenter::GetAvoidVirtualObs() {
  std::shared_lock<std::shared_mutex> lock(data_mutex_);
  return avoid_virtual_obs_;
}
Eigen::Vector3f DataCenter::GetAvoidPlanningVel() {
  std::shared_lock<std::shared_mutex> lock(data_mutex_);
  return avoid_planning_vel_;
}

/****************************************************/
mesh2D DataCenter::GetRobotOutline(const port::CommonPose& robot_pose) {
  mesh2D points = robot::outline_points;
  mesh2D out_points(points.size());
  for (int i = 0; i < points.size(); i++) {
    float local_x = robot_pose.x + points[i][0] * cos(robot_pose.theta) - points[i][1] * sin(robot_pose.theta);
    float local_y = robot_pose.y + points[i][0] * sin(robot_pose.theta) + points[i][1] * cos(robot_pose.theta);
    out_points[i].resize(2);
    out_points[i][0] = local_x;
    out_points[i][i] = local_y;
  }
  return out_points;
}

mesh2D DataCenter::GetRobotDriveWheel(const port::CommonPose& robot_pose, int side) {
  mesh2D points = robot::drive_wheel_points;
  mesh2D out_points(points.size());
  for (int i = 0; i < points.size(); i++) {
    float local_x = points[i][0] * cos(robot_pose.theta) + side * points[i][1] * sin(robot_pose.theta);
    float local_y = points[i][0] * sin(robot_pose.theta) - side * points[i][1] * cos(robot_pose.theta);
    out_points[i].resize(2);
    out_points[i][0] = local_x + robot_pose.x;
    out_points[i][1] = local_y + robot_pose.y;
  }
  return out_points;
}

mesh2D DataCenter::GetRobotOmniWheel(const port::CommonPose& robot_pose, int side, const float& steer_angle) {
  mesh2D points = robot::omni_wheel_points;
  float base_x = robot::omni_wheel_x;
  float base_y = robot::omni_wheel_y;
  float local_x, local_y;
  mesh2D out_points(points.size());
  for (uint i = 0; i < points.size(); i++) {
    //转向轮轮廓车体系坐标
    float x_turn = base_x + points[i][0] * cos(steer_angle) + side * points[i][1] * sin(steer_angle);
    float y_turn = side * base_y - points[i][0] * sin(steer_angle) + side * points[i][1] * cos(steer_angle);
    //车体系转全局系(旋转)
    local_x = x_turn * cos(robot_pose.theta) + y_turn * sin(robot_pose.theta);
    local_y = x_turn * sin(robot_pose.theta) - y_turn * cos(robot_pose.theta);
    //车体系转全局系(平移)
    out_points[i].resize(2);
    out_points[i][0] = local_x + robot_pose.x;
    out_points[i][1] = local_y + robot_pose.y;
  }
  return out_points;
}

vector<mesh2D> DataCenter::GetVisionBoundary(const port::CommonPose& robot_pose) {
  vector<mesh2D> vision_boundary(4);
  vector<mesh2D> orin_vision_boundary(4);
  orin_vision_boundary[0] = robot::front_vision_boundary;
  orin_vision_boundary[l] = robot::back_vision_boundary;
  orin_vision_boundary[2] = robot::left_vision_boundary;
  orin_vision_boundary[3] = robot::right_vision_boundary;
  for (uint i = 0; i < 4; i++) {
    vision_boundary[i].resize(2);
    auto points = orin_vision_boundary[i];
    for (uint j = 0; j < points.size(); j++) {
      float local_x = robot_pose.x + points[j][0] * cos(robot_pose.theta) - points[j][1] * sin(robot_pose.theta);
      float local_y = robot_pose.y + points[j][0] * sin(robot_pose.theta) + points[j][1] * cos(robot_pose.theta);
      vision_boundary[i][0].push_back(local_x);
      vision_boundary[i][1].push_back(local_y);
    }
  }
  return vision_boundary;
}

}  // namespace datacenter
}  // namespace modules
