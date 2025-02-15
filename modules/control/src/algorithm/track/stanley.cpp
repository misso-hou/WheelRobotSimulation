#include "algorithm/track/stanley.h"

#include "tools/math_tools.h"

namespace mathTools = utilities::mathTools;

namespace modules {
namespace control {
namespace algorithm {

port::CommonPose StanleyController::GetTargetPoint(void) {
  port::CommonPose target_pose;
  target_pose.x = target_point_.x;
  target_pose.y = target_point_.y;
  return target_pose;
}

/**
 *@brief:计算路径上目标点(动态控制中心相对路径上最近的点)
 *@param:
 *   index:当前目标点索引
 *   ref_v:规划线速度
 *   maxV:最大线速度
 * @return:
 *   int:目标点索引
 */
int StanleyController::UpdateTargetIndex(const port::CommonPose& robot_pose, const vector<port::CommonPose>& path, const int& index,
                                         const float& ref_v, const float& maxV, const int trim_num) {
  vector<float> dist;  // 机器人到路径点集的距离集合
  /*动态调整前轮中心(类似于预瞄距离)*/
  float frontAxleX = 0;
  float frontAxleY = 0;
  if (maxV < 0) {
    frontAxleX = robot_pose.x - (ref_v / maxV) * 0.4f * cos(robot_pose.theta);
    frontAxleY = robot_pose.y - (ref_v / maxV) * 0.4f * sin(robot_pose.theta);
  } else {
    frontAxleX = robot_pose.x + (ref_v / maxV) * length_ * cos(robot_pose.theta);
    frontAxleY = robot_pose.y + (ref_v / maxV) * length_ * sin(robot_pose.theta);
  }
  /*根据当前目标点位置确定路径上搜索范围*/
  int front_num, back_num;
  if (path.size() < trim_num) {
    front_num = 0;
    back_num = path.size();
  } else {
    front_num = (index - trim_num) > 0 ? (index - trim_num) : 0;
    back_num = index > (int)(path.size() - trim_num) ? (int)path.size() : index + trim_num;
  }
  /*遍历路径上特定范围的点，搜寻目标点(最近点)*/
  for (int i = front_num; i < back_num; i++) {
    dist.push_back(sqrtf(pow(frontAxleX - path[i].x, 2) + pow(frontAxleY - path[i].y, 2)));
  }
  vector<float>::iterator it = find(dist.begin(), dist.end(), *min_element(dist.begin(), dist.end()));
  int index_dis = distance(dist.begin(), it);
  int index_new = front_num + index_dis;
  return index_new;
}

/**
 *@brief:计算跟踪横向误差
 * @return:
 *   error:横向跟踪误差(控制中心与目标点之间的横向偏差)
 */
float StanleyController::CalculateEFA(const port::CommonPose& robot_pose) {
  /*位姿获取*/
  float pX = robot_pose.x;
  float pY = robot_pose.y;
  float pTheta = robot_pose.theta;
  /*目标点获取*/
  float x0 = target_point_.x;
  float y0 = target_point_.y;
  /*计算横向误差*/
  float errorFrontAxle = cos(pTheta) * (pY - y0) - sin(pTheta) * (pX - x0);
  return -errorFrontAxle;
}

/*
 *@brief:计算后轮中心的横向误差
 * @return:
 *   lateral_error:横向跟踪误差(后轮中心)
 */
float StanleyController::CalLateralError(const port::CommonPose& robot_pose, const vector<port::CommonPose>& path, int index, int bias_num) {
  vector<float> dist;
  float lateral_error = 0.0f;
  // 调整路径遍历区间，优化计算
  int start_index = index - bias_num < 0 ? 0 : index - bias_num;
  // start_index = 0;
  if (index > 1) {  //路径刚开始时优先对准方向，不矫正横向误差
    // 搜索跟踪路径上距离后轮中心最近的点
    for (int i = start_index; i < index; i++) {  // todo:可以优化遍历次数
      dist.push_back(sqrtf(pow(robot_pose.x - path[i].x, 2) + pow(robot_pose.y - path[i].y, 2)));
    }
    vector<float>::iterator it = find(dist.begin(), dist.end(), *min_element(dist.begin(), dist.end()));
    int rear_index = start_index + distance(dist.begin(), it);
    rear_index = max(1, rear_index);
    int side = mathTools::WhitchSide(robot_pose, path[rear_index - 1], path[rear_index + 1]);
    lateral_error = side * dist[rear_index - start_index];
  }
  return lateral_error;
}

/*
 *@brief:stanley控制器->转向角速度计算
 *@param:
 *   pose:机器人当前位姿
 *   follow_path:期望路径点集
 *   targetIdx:路径目标点索引
 *   linearV:跟踪线速度
 * @return:
 *   omega:角速度
 */
float StanleyController::StanleyControl(const port::CommonPose& robot_pose, const vector<port::CommonPose>& path, int& targetIdx,
                                        const float& linearV, const float& maxV, const port::PlanType& plan_type) {
  float delta;  //下发角速度
  /*更新目标点，计算期望路径方向*/
  int new_index = UpdateTargetIndex(robot_pose, path, targetId, linearV, maxV, 50);
  targetIdx = new_index;  //目标点更新(外部传值)
  // 路径在目标点处的方向
  float trajectYaw;
  if (new_index == 0) {
    trajectYaw = atan2(path[new_index + 1].y - path[new_index].y, path[new_index + 1].x - path[new_index].x);
  } else {
    trajectYaw = atan2(path[new_index].y - path[new_index - 1].y, path[new_index].x - path[new_index - 1].x);
  }
  /*航向误差计算*/
  float thetaE = 0;
  if (maxV < 0) {
    thetaE = mathTools::NormalizeAngle(trajectYaw - robot_pose.theta + M_PI);  // 倒退航向误差计算
  } else {
    thetaE = mathTools::NormalizeAngle(trajectYaw - robot_pose.theta);  //航向误差计算
  }
  /*横向误差计算*/
  float efa = CalLateralError(robot_pose, path, new_index, 200);  //计算后轮中心对应的横向偏差(设置优化计算参数，无需从头遍历)
  float thetaD = atan2(KE_ * efa, KS_ + abs(linearV));
  /*转向角计算*/
  delta = mathTools::NormalizeAngle(thetaE + thetaD);
  return delta;
}