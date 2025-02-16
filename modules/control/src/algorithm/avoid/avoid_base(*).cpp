#include "algorithm/avoid/avoid_base.h"
#include "tools/math_tools.h"

namespace modules {
namespace control {
namespace algorithm {
namespace avoid_base {

namespace mathTools = utilities::mathTools;

datacenter::DataCenter *DC_Instance = datacenter::DataCenter::GetInstance();

//全局变量定义
port::CommonPose g_robot_pose;
int g_goal_index;
vector<vec2f> g_obs_points;
vector<vec2f> g_boundary;
vector<cv::Point2f> g_cv_boundary;
vector<port::VirtualObs> g_virtual_obs;

//公共变量->速度
vec2f g_cur_cmd;
vec2f g_ref_cmd;
port::Twist g_avoid_cmd;
//公共变量->机身轮廓
port::SampleCircle g_circle;
vector<port::SamplePoint> g_sample_points;
vector<vector<port::ClusterPoint>> g_cluster_groups;
vector<port::CircleAgent> g_agents;
Method g_method;
//用于可视化debug的功能数据(不参与计算)
Eigen::Vector3f g_glanning_vel;
vector<vec2f> g_nearest_points;
vector<vec2f> g_obs_centers;

void ResetAvoidBaseData() {
  g_sample_points.clear();
  g_cluster_groups.clear();
  g_virtual_obs.clear();
  g_nearest_points.clear();
  g_obs_centers.clear();
  for (auto &agent : g_agents) {
    agent.ref_vel_vec = vec2f(0, 0);
    agent.compound_vel = vec2f(0, 0);  // 矢量合成modulate速度
    agent.debug01_vec = vec2f(0, 0);
    agent.debug02_vec = vec2f(0, 0);
    agent.calculated_obstacles.clear();
  }
  // 采样圆位姿更新
  g_circle.global_center = mathTools::LocalToGlobal(g_robot_pose, g_circle.local_center);
  g_circle.heading_dir = mathTools::VecRotateByAngle(g_robot_pose.theta, g_circle.local_center);
  g_circle.heading_dir = g_circle.heading_dir.normalized();
  // 规划速度
  g_planning_vel << 0.0, 0.0, 1.0;
}

void Init(const port::SampleCircle &circle, const vector<port::BoundaryCircle> &circles, const Method &md) {
  // 射线采样圆
  g_circle = circle;
  g_circle.local_center << circle.center_x, circle.center_y;
  g_circle.jacobian << 1.0f, -circle.center_y, 0.0f, circle.center_x;
  // 虚拟轮廓圆
  g_agents.resize(circles.size());
  for (int i = 0; i < circles.size(); i++) {
    // 外部参数设定
    g_agents[i].circle = circles[i];
    g_agents[i].circle.local_center << circles[i].center_x, circles[i].center_y;
    // 常数元素计算
    g_agents[i].jacobian << 1.f, -circles[i].center_y, 0.f, circles[i].center_x;
    g_agents[i].safe_dis = g_agents[i].circle.radius + g_agents[i].circle.safe_margin;
  }
  // 障碍物处理方式
  g_method = md;
}

void UpdateInfo() {
  DC_Instance->SetDsAvoidAgents(g_agents);
  DC_Instance->SetAvoidSamplePoints(g_sample_points);
  DC_Instance->SetAvoidClusterGroup(g_cluster_groups);
  DC_Instance->SetAvoidVirtualObsCenter(g_obs_centers);
  DC_Instance->SetAvoidObsNearestPoints(g_nearest_points);
  DC_Instance->SetAvoidVirtualObs(g_virtual_obs);
  DC_Instance->SetAvoidPlanningVel(g_planning_vel);
}
