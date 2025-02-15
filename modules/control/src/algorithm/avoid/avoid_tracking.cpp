#include "algorithm/avoid/avoid_tracking.h"

#include <opencv2/opencv.hpp>

#include "tools/math_tools.h"

namespace modules {
namespace control {
namespace algorithm {

namespace ab = avoid_base;
namespace mathTools = utilities::mathTools;

void AvoidTracking::SetTrackingPath(const vector<port::CommonPose>& path, const vvec2f& boundary) {
  tracking_path_ = path;
  ab::g_cv_boundary.clear();
  for (auto point : boundary) {
    ab::g_cv_boundary.push_back(cv::Point2f(point(0), point(1)));
  }
}

/**
 * @brief:功能描述:更新跟踪目标点
 * @param:
 *      int:路径点索引
 *      path:沿边路径点集(平滑后的路径)
 *      goal_point: 沿边路径上的目标点
 * @return:控制速度
 */
void AvoidTracking::UpdateGoalPoint(const int goal_index) {
  // 目标点筛选
  for (size_t i = goal_index; i < tracking_path_.size() - 1; ++i) {
    port::CommonPose path_point = tracking_path_[i];
    /*条件一:路径上目标点一定要远离障碍物一定距离*/
    double min_dis = 1.0;
    for (auto obs : ab::g_cluster_groups) {
      //障碍物轮廓点
      vector<cv::Point2f> obs_outline;
      for (auto point : obs) {
        obs_outline.push_back(cv::Point2f(point.pose(0), point.pose(1)));
      }
      //计算路径点相对各障碍物距离
      double dist = cv::pointPolygonTest(obs_outline, cv::Point2f(path_point.x, path_point.y), true);
      dist = min(0.0, dist);  // 过滤障碍物内的点
      min_dis = min(min_dis, fabs(dist));
    }
    if (min_dis < 0.8f) {
      ab::g_goal_index = i;
      continue;
    }
    /*条件二:目标点一定要在机器的正方向*/
    vec2f target_vec(path_point.x - ab::g_circle.global_center(0), path_point.y - ab::g_circle.global_center(1));
    // float target_robot_angle_cos = ab::g_circle.heading_dir.dot(target_vec);
    // if(target_robot_angle_cos <= 0) {
    // ab::g_goal_index = i;
    // continue;
    // }
    /*条件三:远离机器人本体(外接圆)一定距离的目标点*/
    float min_border_distance = target_vec.norm();
    if (ab::g_cluster_groups.size() == 0) {
      if (min_border_distance > 0.2f) {
        ab::g_goal_index = i;
        return;  // 返回满足上述两个条件的目标点
      }
    } else {
      if (min_border_distance > 0.8f) {
        ab::g_goal_index = i;
        return;  // 返回满足上述两个条件的目标点
      }
    }
  }
  return;  // 没有满足条件的目标点，返回上一次的目标点
}

void AvoidTracking::UpdateAgentsVelVecInfo(const int goal_index) {
  UpdateGoalPoint(goal_index);
  // 期望速度计算
  if (tracking_path_.size() > 0 && ab::g_goal_index > 20) {
    port::CommonPose target = tracking_path_[ab::g_goal_index];
    vec2f target_vec(target.x - ab::g_circle.global_center(0), target.y - ab::g_circle.global_center(1));
    target_vec = target_vec.normalized();
    Eigen::Matrix2f rotate_to_local;
    rotate_to_local << cos(ab::g_robot_pose.theta), sin(ab::g_robot_pose.theta), -sin(ab::g_robot_pose.theta), cos(ab::g_robot_pose.theta);
    ab::g_ref_cmd = ab::g_circle.jacobian.inverse() * rotate_to_local * target_vec;
  }
  // agent元素计算
  for (auto& agent : ab::g_agents) {
    // agent圆形轮廓中心全局坐标系位姿
    agent.center_pose = mathTools::LocalToGlobal(ab::g_robot_pose, agent.circle.local_center);
    // agent原始速度坐标系
    agent.cur_vel_vec = agent.jacobian * ab::g_cur_cmd;                                          //当前圆心速度矢量(机体坐标系)
    agent.cur_vel_vec = mathTools::VecRotateByAngle(ab::g_robot_pose.theta, agent.cur_vel_vec);  //圆心速度矢量转全局坐标系
    // 期望速度转速度转agent期望速度矢量
    agent.ref_vel_vec = agent.jacobian * ab::g_ref_cmd;
    agent.ref_vel_vec = mathTools::VecRotateByAngle(ab::g_robot_pose.theta, agent.ref_vel_vec);
    agent.ref_vel_vec = agent.ref_vel_vec.normalized();
  }
}

/**
 * @brief:功能描述:绕障速度包线计算函数(根据方向计算模长)
 * @return:矢量速度模长
 */
float AvoidTracking::CalVelocityEnvelope() {
  float vel_norm = 1.0;
  for (auto agent : ab::g_agents) {
    vec2f v_normal = agent.cur_vel_vec.normalized();
    float obs_v_norm = 1.0f;
    for (auto obs : agent.calculated_obstacles) {
      float angle_ratio = 0.0f;
      float dis_ratio = 1.0f;
      // 计算速度夹角系数
      float local_cos = v_normal.dot(obs.normal_vector);
      // 驶向障碍物
      if (local_cos < 0) {
        float local_ratio = fabs(local_cos);
        angle_ratio = max(angle_ratio, local_ratio);
      }
      // 计算比例距离系数
      float local_dis_ratio = max(0.0f, (obs.nearest_dis - agent.safe_dis) / (ab::g_circle.range - agent.safe_dis));
      dis_ratio = min(dis_ratio, 1.0f - local_dis_ratio);
      // 速度包线设计
      float dis_param = sin(dis_ratio * M_PI / 2);
      float angle_param = sin(angle_ratio * M_PI / 2);
      obs_v_norm = 1.0f - 0.5f * powf(dis_param, 2) - 0.3f * dis_param * powf(angle_param, 2);
      // 最终速度模长确定
      vel_norm = min(vel_norm, obs_v_norm);
      ab::g_planning_vel << dis_param, angle_param, vel_norm;
    }
  }
  return vel_norm;
}

/**
 * @brief:机器位姿控制(机身职能体控制中心)
 * @param:
 *      robot_pose:机器位姿
 *      target:目标位姿
 * @return:控制速度
 */
vec2f AvoidTracking::PoseCtrl(const port::CommonPose& robot_pose, const port::CommonPose& target_pose, const port::Twist& target_cmd) {
  ab::DC_Instance->SetTestTempPose(target_pose);
  /***step01->控制期望计算***/
  //机身虚拟刚体几何参数
  vec2f local_center = ab::g_agents[0].circle.local_center;
  Eigen::Matrix2f jacobian = ab::g_agents[0].jacobian;
  //根据目标相对方位，判断前进还是后退运动
  float proportion_k, feedforward_ratio;  //误差比例系数，前馈速度系数
  auto local_center_target = mathTools::Global2Rob(robot_pose, target_pose);
  if (local_center_target.x > 0.f) {
    //目标位姿在当前机器前方
    proportion_k = 1.0f;
    feedforward_ratio = 0.7f;
  } else {
    // 目标位姿在机器后方，需要切换后面的虚拟刚体质点，否则速度矢量方向向后会导致机器调头，而不是后退
    local_center(0) *= -1;
    jacobian(1, 1) *= -1;
    proportion_k = 1.5f;
    feedforward_ratio = 1.0f;
  }
  port::CommonPose local_center_goal, global_center_goal;
  global_center_goal = mathTools::Rob2Global(target_pose, port::CommonPose(local_center(0), local_center(1), 0.f));
  local_center_goal = mathTools::Global2Rob(robot_pose, global_center_goal);
  /***step02->控制器计算***/
  //前馈补偿(中心参考点期望速度矢量)
  vec2f feedforward_vel_vec(target_cmd.linear, target_cmd.angular);
  feedforward_vel_vec = jacobian * feedforward_vel_vec;
  //误差速度矢量
  vec2f goal_vel_dir(local_center_goal.x - local_center(0), local_center_goal.y - local_center(1));
  vec2f ref_vel = proportion_k * goal_vel_dir + feedforward_ratio * feedforward_vel_vec;
  //航向角误差修正
  float heading_error = mathTools::NormalizeAngle(target_pose.theta - robot_pose.theta);
  vec2f heading_correct_cmd(0.f, 0.5f * heading_error);
  vec2f ctrl_cmd = jacobian.inverse() * ref_vel + heading_correct_cmd;
  //约束线速度大小
  ctrl_cmd(0) = std::max(-1.0f, std::min(1.0f, ctrl_cmd(0)));
  //用于可视化显示
  ab::g_agents[0].center_pose = mathTools::LocalToGlobal(robot_pose, local_center);
  vec2f ctrl_vel = jacobian * ctrl_cmd;
  ctrl_vel = mathTools::VecRotateByAngle(robot_pose.theta, ctrl_vel);
  ab::g_agents[0].ref_vel_vec = ctrl_vel.normalized();
  ab::DC_Instance->SetDsAvoidAgents(ab::g_agents);
  return ctrl_cmd;
}

}  // namespace algorithm
}  // namespace control
}  // namespace modules