#include "algorithm/track/pure_pursuit.h"

#include "tools/math_tools.h"

namespace mathTools = utilities::mathTools;

namespace modules {
namespace control {
namespace algorithm {

/**
 * @brief:计算路径上目标点,获取并更新目标点索引
 * @param:
 *       pose:机器人当前位姿
 *       ref_v:规划线速度
 *       path:跟踪路径
 *       index:当前目标点索引
 * @return:
 *   int:目标点索引
 */
int PurePursuit::GetTargetIndex(const port::CommonPose& pose, const float& ref_v, const vector<port::CommonPose>& path, const int index) {
  vector<float> dist;  // Distance between Vehicle and Trajectory Points
  for (auto point : path) {
    dist.push_back(sqrtf(pow(pose.x - point.x, 2) + pow(pose.y - point.y, 2)));
  }
  vector<float>::iterator it = find(dist.begin(), dist.end(), *min_element(dist.begin(), dist.end()));

  int index_new = distance(dist.begin(), it);
  index_new = max(index, index_new);
  if (index_new - index > 20) index_new = index;  //路径最近点索引
  float length = dist[index_new];
  // float newlD = (klD_ * ref_v) + lD_; // 新的预瞄距离计算
  // float new_ld = (ref_v / (2.0	* max_v)) * ld_;  // note:保证高速预瞄距离尽可能远,弯道(低速)预瞄准距离可以降到足够小
  float new_ld = (ref_v / max_v_) * ld_;
  new_ld = min(ld_, new_ld);
  while ((new_ld > length) && (index_new < path.size() - 1)) {
    float x0 = path[index_new].x;
    float y0 = path[index_new].y;
    index_new++;                                                // update goal point index within the look ahead distance
    length = sqrtf(pow(x0 - pose.x, 2) + pow(y0 - pose.y, 2));  //更新目标点相对后轮中心距离
  }
  return index_new;
}

/**
 * @brief:计算路径上目标点,获取并更新目标点索引
 * @param:
 *   pose:机器人当前位姿
 *   linear_v:规划线速度
 *   targetIdx:目标点索引
 * @return:
 *   omega:控制角速度
 */
float PurePursuit::PurePursuitControl(const port::CommonPose& pose, const float& linear_v, const vector<port::CommonPose>& path, int& targetIdx) {
  int index_new = GetTargetIndex(pose, linear_v, path, targetIdx);
  targetIdx = index_new;
  port::CommonPose target = (index_new < path.size() ? path[index_new] : path.back());  // 目标点获取
  angAlpha_ = atan2(target.y - pose.y, target.x - pose.x);
  float thetaE = mathTools::NormalizeAngle(angAlpha_ - pose.theta);  //航向偏差
  float dist_to_goal = hypotf(target.x - pose.x, target.y - pose.y);
  float R = dist_to_goal * 0.5f / sin(thetaE);
  float omega = 0.0f;
  if (dist_to_goal > 0.2f) {
    omega = linear_v / R;
  } else {
    if (path.size() >= 2) {
      angAlpha_ = atan2(path.back().y - path[path.size() - 2].y, path.back().x - path[path.size() - 2].x);
      thetaE = mathTools::NormalizeAngle(angAlpha_ - pose.theta);
      omega = thetaE;
    }
  }
  return omega;
}

}  // namespace algorithm
}  // namespace control
}  // namespace modules
