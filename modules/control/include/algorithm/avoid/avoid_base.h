#pragma once
#include <Eigen/Dense>
#include <iostream>

#include "common_port/data_port.h"
#include "control_base.h"
#include "data_center.h"

namespace modules {
namespace control {
namespace algorithm {
namespace avoid_base {

enum Method { POLYGON = 0, INTERPOLATE };

//简化命名空间
namespace port = utilities::port;

//简化变量名
using vec2f = Eigen::Vector2f;
using mat2f = Eigen::Matrix2f;
using vvcp = vector<vector<port::ClusterPoint>>;

extern datacenter::DataCenter* DC_Instance;

//公共变量->障碍
extern port::CommonPose g_robot_pose;
extern int g_goal_index;                        //避障模块路径上目标点索引
extern vector<vec2f> g_obs_points;              //原始障碍物点云
extern vector<vec2f> g_boundary;                //地图边界
extern vector<cv::Point2f> g_cv_boundary;       // cv类型边界
extern vector<port::VirtualObs> g_virtual_obs;  //后处理障碍物

//公共变量->速度
extern vec2f g_cur_cmd;          //控制中心当前速度
extern vec2f g_ref_cmd;          //控制中心期望速度
extern port::Twist g_avoid_cmd;  // 避障碍速度
//公共变量->机身轮廓
extern port::SampleCircle g_circle;  // 障碍物点采样圆形轮廓
// FIXME:这个变量目前避障跟跟踪模块还在使用，后面应该只给动态系统使用
extern vector<port::SamplePoint> g_sample_points;
extern vector<vector<port::ClusterPoint>> g_cluster_groups;  //聚类障碍物点云数据
extern vector<port::CircleAgent> g_agents;                   //机身单元
extern Method g_method;
//用于可视化debug的功能数据(不参与计算)
extern Eigen::Vector3f g_planning_vel;  // 规划速度曲面显示
extern vector<vec2f> g_nearest_points;  // debug障碍物最近点
extern vector<vec2f> g_obs_centers;

//基础功能函数
void ResetAvoidBaseData();
void Init(const port::SampleCircle& circle, const vector<port::BoundaryCircle>& circles, const Method& md);
void UpdateInfo();

}  // namespace avoid_base
}  // namespace algorithm
}  // namespace control
}  // namespace modules
