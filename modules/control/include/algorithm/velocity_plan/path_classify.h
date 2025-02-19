#pragma once
#include <cmath>
#include <iostream>
#include <string.h>
#include <vector>

#include "common_port/data_port.h"

namespace modules {
namespace control {
namespace algorithm {

using namespace std;
namespace port = utilities::port;

class PathClassify {
 private:
  float friction_factor_;               //摩擦系数，约束车辆过弯最大速度
  float compound_curve_limit_v_ = 0.3;  // 复杂曲线的跟踪速度约束
  float length_thd_;                    //短/长直线定义阈值
  float angle_thd_;                     //曲线拐点定义阈值
  int adjust_index_;                    //过短曲线点(方向突变杂点)过滤阈值
  float min_curve_v_;                   //最小曲线速度约束
  //分类路径数据
  vector<port::PathCurve> curves_;
  vector<port::DangerousBorder> dg_borders_;
  vector<port::CommonPose> plt_curves_;
  vector<port::CommonPose> plt_dg_borders_;
  port::PathCurve proximate_curve_;
  port::DangerousBorder proximate_dg_border_;
  vector<port::TrajPoint> traj_path_;

 public:
  PathClassify(float friction, float length, float angle, int num, float min_v) {
    friction_factor_ = friction;
    length_thd_ = length;
    angle_thd_ = angle;
    adjust_index_ = num;
    min_curve_v_ = min_v;
  }
  PathClassify() {}
  ~PathClassify(){};

 private:
  vector<port::DangerousBorder> DangerousBorderClassify(const vector<port::CommonPose>& smooth_path);
  vector<port::PathCurve> CurveClassify(const vector<port::CommonPose>& smooth_path);
  vector<port::PathCurve> ExtractSharpCurves(vector<port::PathCurve>& curves);
  void CalSharpCurveV(vector<port::PathCurve>& curves);
  void ConvertPltPath(const vector<port::CommonPose>& path);
  void PlanningTrajectory(const vector<port::CommonPose>& path);

 public:
  pair<port::DangerousBorder, port::PathCurve> GetProximateSpecialPath(int nearest_index);
  pair<vector<port::CommonPose>, vector<port::CommonPose>> GetPltSpecialPath();
  vector<port::TrajPoint> GetPlanningTrajectory();
  void PathDispose(const vector<port::CommonPose>& orin_path);
};

}  // namespace algorithm
}  // namespace control
}  // namespace modules
