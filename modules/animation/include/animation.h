#pragma once

#include <iostream>
#include <matplotlibcpp17/axes.h>
#include <matplotlibcpp17/figure.h>
#include <matplotlibcpp17/pyplot.h>
#include <memory>
#include <pybind11/embed.h>
#include <pybind11/pybind11.h>
#include <vector>

#include "common_port/data_port.h"
#include "facilities/singleton.h"

#include "facilities/base_time_struct. h"

namespace py = pybind11;
namespace mpl = matplotlibcpp17;
namespace port = utilities::port;

namespace modules {
namespace animation {

typedef enum { SIMULATION = 0, LOGGER } Mode;

typedef enum { ENV = 0, SENSOR } Scene;

using mesh2D = vector<vector<float>>;

class Animation : public utilities::Singleton<Animation> {
  friend class Singleton<Animation>;

 private:
  Animation() {}
  ~Animation() {}

 private:
  void EnvPltInit(int ratio, const pybind11::dict& fig_kwargs, const port::CommonPose& robot_pose);
  void MapPltInit(const pybind11::dict& fig_kwargs, int ratio, const port::CommonPose& robot_pose);
  void CmdPltInit(const pybind11::dict& fig_kwargs, const float& x_axis_range);
  void SensorPltInit(const pybind11::dict& fig_kwargs, const float& offset);
  void SpacePltInit(const pybind11::dict& fig_kwargs);

 public:
  void InitializePlt(const port::CommonPose& robot_pose, const Mode& mode);
  void AnimationLoop();

 private:
  //画框
  mpl::pyplot::PyPlot env_plt_;
  mpl::pyplot::PyPlot cmd_plt_;
  mpl::pyplot::PyPlot map_plt_;
  mpl::pyplot::PyPlot sensor_plt_;
  mpl::pyplot::PyPlot space_plt_;
  //轴系
  shared_ptr<mpl::axes::Axes> env_axes_ptr_;     //主图轴系
  shared_ptr<mpl::axes::Axes> cmd_axes_ptr_;     //速度监视器轴系
  shared_ptr<mpl::axes::Axes> map_axes_ptr_;     //地图图轴系
  shared_ptr<mpl::axes::Axes> sensor_axes_ptr_;  //传感器监视器轴系
  shared_ptr<mpl::axes::Axes> space_axes_ptr_;   //传感器监视器轴系
  // figure
  shared_ptr<mpl::figure::Figure> env_figure_ptr_;
  shared_ptr<mpl::figure::Figure> cmd_figure_ptr_;
  shared_ptr<mpl::figure::Figure> map_figure_ptr_;
  shared_ptr<mpl::figure::Figure> sensor_figure_ptr_;
  shared_ptr<mpl::figure::Figure> space_figure_ptr_;
  // background
  py::object env_background_;
  py::object env_base_background_;
  py::object cmd_background_;
  py::object map_background_;
  py::object sensor_background_;
  py::object space_background_;  //颜色映射
  py::object jet_cmap_;
  //标志
  bool map_init_flag_;

 private:  //数据获取函数
  void GetCommonData();

 private:  //内部处理函数
  py::object DefinePoly(const vector<vector<float>>& points, string color, float alpha);
  void SetEnvAxisLimit(const mesh2D& traj, const int ratio, float offset);
  bool FrequencyCtrl(int T, int64_t& last_time_stamp);
  vector<Eigen::Vector2f> ObsDataTrans(const vector<float>& obs_data);

 private:  //可视化函数
  void PlotSensorObs(const Scene& scene);
  void PlotGridMapObs();
  void PlotEntityObs(const Scene& scene);
  //公共数据显示(环境数据)
  void Plotline();
  //机器相关数据显示函数
  void PlotRobotOutline(const Scene& scene);
  void PlotLidarData();
  void PlotVisionBoundary();
  void PlotPointsOnPath();
  void PlotTargetOutline();
  // 算法相关特定数据显示
  // mpc
  void PlotMPCHrizon();
  //动态系统避障
  void DynSysAvoidDisplay();
  void PlotAvoidAgents(const vector<port::CircleAgent>& avoid_agents);
  void PlotAvoidAgentsConvex();
  void PlotObsSandCPoints();
  void PlotC1VirtualObs();
  void PlotAgentVelVector(const port::CircleAgent& agent);
  void PlotAgentObsQuiver(const port::CircleAgent& agent);

 public:
  void SetTrackingData(const port::CommonPose& robot_pose, const port::Twist& cmd, const mesh2D& path, const mesh2D& sp_curve,
                       const mesh2D& dg_border, const port::CommonPose& target, const bool erase, const bool blade);

  void SetObsData(const port::CommonPose& sync_pose, const vector<float>& sensor_obs, const vector<float>& map_obs);

  void EnvironmentDisplay();
  void CmdMonitor(int buffer_length);
  void MapDisplay();
  void ObsMonitor();
  void SpaceRobot(const float& rate);
  //速度规划显示
  void SurfaceVelPlanning(int T);

 private:  //显示数据
  Mode running_mode_;
  port::CommonPose robot_pose_;
  port::Twist cmd_;
  mesh2D tracking_path_;
  mesh2D trajectory_;
  mesh2D sharp_curve_;
  mesh2D dg_border_;
  mesh2D map_boundary_;
  vector<mesh2D> plot_line_;
  port::CommonPose target_;
  vector<pair<vector<float>, vector<float>>> orin_obs_;
  vector<Eigen::Vector2f> sensor_obs_;
  vector<Eigen::Vector2f> map_obs_;
  port::CommonPose sync_pose_;
  bool blade_;
  bool axis_change_;
  bool erase_flag_;
  // vector<port: :CommonPose> mpc_horizon_; //todo:数据回放也需要显示
  //动态系统	//todo：动态系统相关数据也需要显示
};
}  // namespace animation
}  // namespace modules
