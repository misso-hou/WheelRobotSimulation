#include "algorithm/avoid/deal_with_obs.h"

#include <numeric>
#include <opencv2/opencv.hpp>

#include "tools/math_tools.h"

namespace modules {
namespace control {
namespace algorithm {

namespace ab = avoid_base;
namespace mathTools = utilities::mathTools;

DealWithObs::DealWithObs(int pmn, const float& eps, const float& idis) {
  points_min_num_ = pmn;
  epsilon_ = eps;
  interpolate_dis_ = idis;
}

/**
 * @brief:提取边界内的障碍物
 */
vvec2f DealWithObs::ExtractInBoundaryObs(const vvec2f& obs_points) {
  vvec2f in_boundary_obs = obs_points;
  // for (auto obs_point : obs_points) {
  //   double result = cv::pointPolygonTest(ab::g_cv_boundary, cv::Point2f(obs_point(0), obs_point(1)), false);
  //   //边界内障碍物
  //   if (result > 0) {
  //     in_boundary_obs.push_back(obs_point);
  //   }
  // }
  return in_boundary_obs;
}

/**
 * @brief: 机器障碍物点云采样计算(按角度分组，计算360度范围内的最近点)
 * @param:
 */
int DealWithObs::CalSampledPoints(const vvec2f& obs_points) {
  // 角度区间分组
  int group_num = ab::g_circle.sub_num;
  float sample_angle = 2 * M_PI / group_num;  //采样角度
  vector<vector<port::SamplePoint>> subgroup(group_num);
  /***按角度分组***/
  for (auto point : obs_points) {
    // 中心->点向量
    vec2f local_point = point - ab::g_circle.global_center;
    float polar_r = local_point.norm();  //极坐标半径
    // 极坐标方位角计算
    float polar_theta = mathTools::NormalizeAngle(atan2f(local_point(1), local_point(0)) - ab::g_robot_pose.theta);
    if (polar_theta < 0) {
      polar_theta += 2 * M_PI;
    }
    int subgroup_index = (int)(polar_theta / sample_angle);
    subgroup_index = min(subgroup_index, group_num - 1);
    port::SamplePoint new_point;
    new_point.pose = point;
    new_point.polar_dis = polar_r;
    new_point.phase = (int)(polar_theta / (M_PI / 2)) + 1;
    new_point.angle = polar_theta * 180 / M_PI;
    // 插入新的采样点
    subgroup[subgroup_index].push_back(new_point);
  }
  /***计算各角度区间内最近的障碍物点***/
  // 分组遍历(每组内选出一个最近点)
  for (int i = 0; i < group_num; i++) {
    if (subgroup[i].empty()) continue;
    float min_dis = ab::g_circle.range;
    port::SamplePoint sub_sample_point;
    sub_sample_point.pose << 0.f, 0.f;
    // 寻找组内最近点
    for (auto subgroup_point : subgroup[i]) {
      //过滤大于采样距离外的点，且角度区间内只保留最近点
      if (subgroup_point.polar_dis < min_dis) {
        min_dis = subgroup_point.polar_dis;
        sub_sample_point = subgroup_point;
      }
    }
    // 过滤空值
    if (sub_sample_point.pose.norm() > 0.f) {
      ab::g_sample_points.push_back(sub_sample_point);
    }
  }
  if (ab::g_sample_points.size() < points_min_num_) {
    return 0;
  } else {
    return 1;
  }
}

vvcp DealWithObs::SamplePointsCluster(int min_pt_num, const float& dis_thd) {
  // 提取采样点坐标，用于聚类
  vector<vec2f> sample_points_pose(ab::g_sample_points.size());
  for (int i = 0; i < ab::g_sample_points.size(); i++) {
    sample_points_pose[i] = ab::g_sample_points[i].pose;
  }
  // 采样点聚类处理
  auto cluster_points = dbscan_.dbscanFit(min_pt_num, dis_thd, sample_points_pose);
  vector<port::ClusterPoint> c_points = cluster_points.first;  // 聚类点集
  int cluster_num = cluster_points.second;                     // 聚类个数
  vector<vector<port::ClusterPoint>> cluster_groups(cluster_num);
  // 聚类结果为空(滤除噪点信息)
  if (cluster_num == 0) return cluster_groups;
  /***虚拟障碍物计算***/
  // 聚类点分组
  for (auto point : c_points) {
    // 筛选类组内的点
    if (point.clusterID > 0) {
      // 分组采样点提取
      for (int i = 1; i <= cluster_num; i++) {
        if (point.clusterID == i) {
          cluster_groups[i - 1].push_back(point);
        }
      }
    }
  }
  // 跨相限聚类点重新排序(1和4相限点同时存在)
  for (auto& group : cluster_groups) {
    int phase_1 = 0;
    int phase_2 = 0;
    int phase_4 = 0;
    for (auto point : group) {
      int point_phase = ab::g_sample_points[point.index].phase;
      if (point_phase == 1) {
        phase_1++;
      } else {
        if (phase_1 == 0) {
          break;  // 无第一相限点,不存在跨相限情况
        } else {
          if (point_phase == 2) {
            phase_2++;
          } else if (point_phase == 4) {
            phase_4++;
          }
        }
      }
    }
    // 集合只有1和4相限点需要重新排序
    if (phase_4 > 0) {
      // 三种跨相限情况(1&4,1&2&4,1&3&4)
      if (phase_2 == 0) {
        rotate(group.begin(), group.begin() + phase_1, group.end());
      } else {
        rotate(group.begin(), group.begin() + phase_1 + phase_2, group.end());
      }
    }
  }
  return cluster_groups;
}

vvcp DealWithObs::ObsPointsConnect(const vvcp& cluster_group, const float& connect_thd) {
  int cluster_num = cluster_group.size();
  vector<vector<port::ClusterPoint>> copy_cluster = cluster_group;
  // 障碍物拼接
  vector<int> connet_id(cluster_num);
  // 判断各障碍物的连接情况(1->相接，0->中断）
  for (int i = 0; i < cluster_num; i++) {
    port::ClusterPoint end_point, front_point;
    if (i < cluster_num - 1) {
      //计算障碍物之间首尾距离
      end_point = cluster_group[i].back();
      front_point = cluster_group[i + 1].front();
    } else {
      end_point = cluster_group[cluster_num - 1].back();
      front_point = cluster_group[0].front();
    }
    float bias_dis = (front_point.pose - end_point.pose).norm();
    if (bias_dis > connect_thd) {
      connet_id[i] = 0;
    } else {
      connet_id[i] = 1;
    }
  }
  // 障碍物拼接操作
  vector<vector<port::ClusterPoint>> connected_obs_points;
  vector<port::ClusterPoint> local_obs_points = cluster_group[0];
  for (int i = 0; i < cluster_num - 1; i++) {
    // 中断障碍物
    if (connet_id[i] == 0) {
      connected_obs_points.push_back(local_obs_points);
      local_obs_points = cluster_group[i + 1];
    }
    // 拼接操作
    else {
      local_obs_points.insert(local_obs_points.end(), copy_cluster[i + 1].begin(), copy_cluster[i + 1].end());
    }
  }
  // 最后一个障碍与第一个相接
  if (connet_id[cluster_num - 1] == 1) {
    int sum = accumulate(connet_id.begin(), connet_id.end(), 0);
    // 所有的障碍物首尾相接
    if (sum == cluster_num) {
      connected_obs_points.push_back(local_obs_points);
    }
    // 将后面的未处理的障碍物和第一个提取出来的障碍物进行合并
    else {
      connected_obs_points.front().insert(connected_obs_points.front().begin(), local_obs_points.begin(), local_obs_points.end());
    }
  }
  // 第一个和最后一个障碍物首尾不相接
  else {
    connected_obs_points.push_back(local_obs_points);
  }
  return connected_obs_points;
}

vector<port::VirtualObs> DealWithObs::ObsPointsInterpolation(const vvcp& cluster_obs) {
  int obs_num = cluster_obs.size();
  vector<port::VirtualObs> obstacles(obs_num);
  // 对虚拟障碍物点进行插值处理
  for (int i = 0; i < obs_num; i++) {
    // 提取障碍物点坐标
    vector<float> obs_x, obs_y;
    for (auto point : cluster_obs[i]) {
      obs_x.push_back(point.pose(0));
      obs_y.push_back(point.pose(1));
    }
    vector<float> spline_xs, spline_ys;
    // // 不进行插值，使用使用原始点
    // spline_xs = obs_x;
    // spline_ys = obs_y;
    // // 直线插值
    // std::tie(spline_xs, spline_ys) = itp_tools_.LineInterpolate(obs_x, obs_y, interpolate_dis_);
    // 样条插值
    std::tie(spline_xs, spline_ys) = itp_tools_.InterpolateXY(obs_x, obs_y, interpolate_dis_);
    // 插值障碍物点获取
    int ipt_points_num = spline_xs.size();
    obstacles[i].points.resize(ipt_points_num);
    for (int j = 0; j < ipt_points_num; j++) {
      vec2f obs_points(spline_xs[j], spline_ys[j]);
      obstacles[i].points[j] = obs_points;
    }
  }
  return obstacles;
}

int DealWithObs::CalVirtualITPLObs(const float& connect_thd) {
  // step01->采样点聚类
  auto cluster_groups = SamplePointsCluster(points_min_num_, epsilon_);
  if (cluster_groups.size() == 0) return 0;
  // step02->聚类点二次处理(拼接)
  ab::g_cluster_groups = ObsPointsConnect(cluster_groups, connect_thd);
  // step03->特征点插值获取虚拟障碍物轮廓点集
  ab::g_virtual_obs = ObsPointsInterpolation(ab::g_cluster_groups);
  ab::g_obs_centers.resize(ab::g_virtual_obs.size());
  // step04->计算障碍物几何中心
  for (int i = 0; i < ab::g_virtual_obs.size(); i++) {
    vec2f points_vec_sum(0.0f, 0.0f);
    for (auto point : ab::g_virtual_obs[i].points) {
      points_vec_sum += point;
    }
    ab::g_virtual_obs[i].center = points_vec_sum / ab::g_virtual_obs[i].points.size();
    ab::g_obs_centers[i] = ab::g_virtual_obs[i].center;
  }
  return 1;
}

vvcp DealWithObs::ConvexClusterGroup(const vvcp& cluster_group) {
  vvcp ConvexGroups(cluster_group.size());
  for (int i = 0; i < cluster_group.size(); i++) {
    //提取聚类点
    vector<cv::Point2f> points;
    for (auto ct_point : cluster_group[i]) {
      points.push_back(cv::Point2f(ct_point.pose(0), ct_point.pose(1)));
    }
    // 提取凸包
    vector<int> hullIndices;
    cv::convexHull(points, hullIndices, false, true);
    // 数据转录
    for (auto id : hullIndices) {
      ConvexGroups[i].push_back(cluster_group[i][id]);
    }
    ConvexGroups[i].push_back(ConvexGroups[i].front());
  }
  return ConvexGroups;
}

int DealWithObs::CalVirtualVertexObs() {
  // step01->采样点聚类
  auto cluster_groups = SamplePointsCluster(points_min_num_, epsilon_);
  if (cluster_groups.size() == 0) return 0;
  // step02->聚类点凸包点
  ab::g_cluster_groups = ConvexClusterGroup(cluster_groups);
  // step03->凸包障碍物提取(凸点，边向量，几何中心)
  vector<port::VirtualObs> convex_obs(ab::g_cluster_groups.size());
  ab::g_obs_centers.resize(ab::g_cluster_groups.size());
  for (int i = 0; i < ab::g_cluster_groups.size(); i++) {
    int points_num = ab::g_cluster_groups[i].size();
    convex_obs[i].points.resize(points_num);
    convex_obs[i].sides.resize(points_num);
    convex_obs[i].center = vec2f(0.0f, 0.0f);
    for (int j = 0; j < points_num; j++) {
      //凸点提取
      convex_obs[i].points[j] = ab::g_cluster_groups[i][j].pose;
      //凸边计算
      if (j < points_num - 1) {
        convex_obs[i].sides[j] = ab::g_cluster_groups[i][j + 1].pose - ab::g_cluster_groups[i][j].pose;
      } else {
        convex_obs[i].sides[j] = ab::g_cluster_groups[i][0].pose - ab::g_cluster_groups[i][j].pose;
      }
      // 几何中心计算
      convex_obs[i].center += ab::g_cluster_groups[i][j].pose;
    }
    convex_obs[i].center /= points_num;
    ab::g_obs_centers[i] = convex_obs[i].center;
  }
  ab::g_virtual_obs = convex_obs;
  return 1;
}

pair<vec2f, float> DealWithObs::CalPlygonObsNormal(const vec2f& focus, const port::VirtualObs& obstacle) {
  int points_num = obstacle.points.size();
  vector<float> side_min_dis(points_num);
  vector<vec2f> side_normal_vec(points_num);
  for (int i = 0; i < points_num; i++) {
    vec2f relative_vec = focus - obstacle.points[i];
    float relative_side_angle = relative_vec.dot(obstacle.sides[i]);
    if (relative_side_angle <= 0) {
      side_min_dis[i] = relative_vec.norm();
      side_normal_vec[i] = relative_vec;
    } else {
      float project_ratio = relative_vec.dot(obstacle.sides[i]) / obstacle.sides[i].dot(obstacle.sides[i]);
      // 过滤垂点不在polygon side上的情况(垂点在polygon外部)
      if (project_ratio > 1) {
        side_min_dis[i] = relative_vec.norm();
        side_normal_vec[i] = relative_vec;
      } else {
        side_normal_vec[i] = relative_vec - project_ratio * obstacle.sides[i];
        side_min_dis[i] = side_normal_vec[i].norm();
      }
    }
  }
  //计算最小距离
  auto it = min_element(side_min_dis.begin(), side_min_dis.end());
  int min_id = distance(side_min_dis.begin(), it);
  return make_pair(side_normal_vec[min_id], *it);
}

pair<vec2f, float> DealWithObs::CalPointsCloudNormal(const vec2f& focus, const port::VirtualObs& obstacle) {
  int points_num = obstacle.points.size();
  vector<float> point_dis(points_num);
  for (int i = 0; i < points_num; i++) {
    vec2f point_vec = focus - obstacle.points[i];
    point_dis[i] = point_vec.norm();
  }
  //计算最小距离
  auto it = min_element(point_dis.begin(), point_dis.end());
  int min_id = distance(point_dis.begin(), it);
  return make_pair(focus - obstacle.points[min_id], *it);
}

pair<vec2f, float> DealWithObs::CalObsNormalVec(const vec2f& pose, const port::VirtualObs& v_obs) {
  vec2f obs_normal;
  float min_dis;
  if (ab::g_method == ab::Method::POLYGON) {
    tie(obs_normal, min_dis) = CalPlygonObsNormal(pose, v_obs);
  } else if (ab::g_method == ab::Method::INTERPOLATE) {
    tie(obs_normal, min_dis) = CalPointsCloudNormal(pose, v_obs);
  }
  return make_pair(obs_normal, min_dis);
}

vector<port::CalculatedObs> DealWithObs::ComputeAgentsCalObs(const port::CircleAgent& agent) {
  vector<port::CalculatedObs> calculated_obstacles(ab::g_virtual_obs.size());
  // 虚拟障碍物向量元素计算
  for (int i = 0; i < ab::g_virtual_obs.size(); i++) {
    auto [obs_normal, min_dis] = CalObsNormalVec(agent.center_pose, ab::g_virtual_obs[i]);
    float ratio_dis = min_dis / ab::g_circle.range;
    ratio_dis = ratio_dis < agent.safe_dis / ab::g_circle.range ? 0.0f : ratio_dis;
    ab::g_virtual_obs[i].min_ratio_dis = ab::g_virtual_obs[i].min_ratio_dis > 0.f ? min(ab::g_virtual_obs[i].min_ratio_dis, ratio_dis) : ratio_dis;
    // 法向量和切向量元素计算
    calculated_obstacles[i].nearest_point = agent.center_pose - obs_normal;
    ab::g_nearest_points.push_back(calculated_obstacles[i].nearest_point);
    vec2f normal_vector = obs_normal.normalized();
    vec2f tangent_vector(normal_vector(1), -normal_vector(0));
    calculated_obstacles[i].normal_vector = normal_vector;
    calculated_obstacles[i].tangent_vector = tangent_vector;
    // 最近障碍物距离
    calculated_obstacles[i].nearest_dis = min_dis;
    calculated_obstacles[i].reference_point = ab::g_virtual_obs[i].center;
    // 障碍物碰撞检测
    if (min_dis < agent.safe_dis - agent.circle.safe_margin) {
      cout << "obstacle crash!!!" << endl;
    }
  }
  return calculated_obstacles;
}

int DealWithObs::DisposeObs(const vvec2f& obs_points) {
  int result = 0;
  /*step01->提取边界内障碍*/
  auto valid_obs = ExtractInBoundaryObs(obs_points);
  /*step02->射线法扫描临近障碍物点*/
  if (CalSampledPoints(valid_obs) == 0) return result;
  /*step03->虚拟障碍物计算*/
  if (ab::g_method == ab::Method::POLYGON) {
    result = CalVirtualVertexObs();
  } else if (ab::g_method == ab::Method::INTERPOLATE) {
    result = CalVirtualITPLObs(1.5f);
  }
  /*step04->“计算障碍物"计算(带有附加信息的障碍物)*/
  for (auto& agent : ab::g_agents) {
    agent.calculated_obstacles = ComputeAgentsCalObs(agent);
  }
  return result;
}

}  // namespace algorithm
}  // namespace control
}  // namespace modules