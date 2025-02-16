#include "algorithm/avoid/dynamic_system.h"

#include "tools/math_tools.h"

namespace modules {
namespace control {
namespace algorithm {

namespace ab = avoid_base;
namespace mathTools = utilities::mathTools;

DynamicSys::DynamicSys(const float& rf, const int sf, const float& cd, const float& wp) {
  react_factor_ = rf;  // 流场响应系数
  safe_factor_ = sf;   //突破安全边界保护系数
  critical_dis_ = cd;  //多障碍物决断距离
  weight_power_ = wp;  //多障碍物影响因子
}

vec2f DynamicSys::LimitIntersectionAngle(const vec2f& benchmark_vec, const vec2f& object_vec, const float& angle_thd) {
  vec2f benchmark = benchmark_vec.normalized();
  vec2f benchmark_tangent = vec2f(benchmark(1), -benchmark(0));
  vec2f object = object_vec.normalized();
  float intersection_angle = acosf(benchmark.dot(object));
  vec2f new_vec = object;
  float angle_abs = fabs(intersection_angle);
  if (angle_abs > angle_thd) {
    float angle_sign = angle_abs / intersection_angle;
    float new_angle = (M_PI - angle_abs) / (M_PI - angle_thd) * angle_thd;
    // new_angle *= angle_sign;
    if (object.dot(benchmark_tangent) > 0) {
      new_vec = mathTools::VecRotateByAngle(-new_angle, benchmark);
    } else {
      new_vec = mathTools::VecRotateByAngle(new_angle, benchmark);
    }
    new_vec = new_vec.normalized();
  }
  return new_vec;
}

/**
 * @brief:功能描述:根据到边界距离判断绕障方向
 */
port::AvoidDir DynamicSys::CalAvoidDir(const port::VirtualObs& vt_obs) {
  port::AvoidDir dir = port::AvoidDir::random;
  //计算障碍物到边界距离
  double min_dis = 2.0;
  for (auto point : vt_obs.points) {
    //计算路径点相对各障碍物距离
    double dist = cv::pointPolygonTest(ab::g_cv_boundary, cv::Point2f(point(0), point(1)), true);
    min_dis = min(min_dis, fabs(dist));
  }
  if (min_dis < 1.5f) {
    dir = port::AvoidDir::left;
  }
  return dir;
}

void DynamicSys::CalAgentsCalObsBasicInfo(port::CircleAgent& agent) {
  //虚拟障碍物向量元素计算
  for (uint i = 0; i < agent.calculated_obstacles.size(); ++i) {
    port::CalculatedObs cal_obs = agent.calculated_obstacles[i];
    // reference元素计算
    cal_obs.reference_vector = (agent.center_pose - cal_obs.reference_point).normalized();  // 障碍物referece vector计算
    // 特征距离计算
    cal_obs.gamma_dis = cal_obs.nearest_dis / agent.safe_dis;
    // cal_obs.gamma_dis = CalObsGammaDis(-normal_vector, min_dis);
    // 正交矩阵计算
    cal_obs.orthogonal_matrix << cal_obs.normal_vector, cal_obs.tangent_vector;
    cal_obs.basic_matrix << cal_obs.reference_vector, cal_obs.tangent_vector;
    cal_obs.avoid_dir = CalAvoidDir(ab::g_virtual_obs[i]);
    //更新计算障碍物
    agent.calculated_obstacles[i] = cal_obs;
  }
}

/**
 * @brief:障碍物权重计算
 * @param:
 *      obstacles:圆形机器单元对应的计算障碍物
 */
void DynamicSys::CalObsWeights(vector<port::CalculatedObs>& obstacles) {
  int obs_num = obstacles.size();
  Eigen::ArrayXf weights(obs_num);
  Eigen::ArrayXf critical_obstacles(obs_num);
  Eigen::ArrayXf obs_nearest_dis(obs_num);
  Eigen::ArrayXf criticle_dis_array(obs_num);
  bool critical_flag = false;
  vector<float> obs_dis_weights(obs_num);
  /***step01->决断权重计算(优先考虑决断距离内的障碍物)***/
  // 筛选决断距离内的关键障碍物
  int sum_critical_obstacles = 0;
  for (int i = 0; i < obs_num; ++i) {
    obs_nearest_dis(i) = obstacles[i].nearest_dis;
    criticle_dis_array(i) = CRITICAL_DIS;
    int tmp_value = (obstacles[i].nearest_dis <= CRITICAL_DIS) ? 1 : 0;
    critical_obstacles(i) = tmp_value;
    sum_critical_obstacles += tmp_value;
  }
  // 障碍物决断权重计算
  if (sum_critical_obstacles > 0) {
    critical_flag = true;
    weights = critical_obstacles / sum_critical_obstacles;
  } else {
    weights = Eigen::pow(obs_nearest_dis - CRITICAL_DIS, -WEIGHT_POWER);
    weights /= weights.sum();
  }
  /***step02->距离权重计算***/
  int safe_divide = 0;
  for (auto obs : obstacles) {
  }
  // 计算障碍物对应权重
  for (int k = 0; k < obs_num; k++) {
    float gamma_k = max(0.0f, obstacles[k].gamma_dis - 1.0f);
    if (gamma_k == 0.f) safe_divide++;  // NOTE:float使用等号判断
    obs_dis_weights[k] = 1.0f;
    for (int i = 0; i < obs_num; i++) {
      if (i == k) continue;
      float gamma_i = max(0.0f, obstacles[i].gamma_dis - 1.0f);
      if (gamma_k < 0.001f && gamma_i < 0.001f) {
        obs_dis_weights[k] = 1.0f;
      } else {
        obs_dis_weights[k] *= gamma_i / (gamma_k + gamma_i);
      }
    }
  }
  // 单一障碍物处理
  if (obs_num == 1) {
    obs_dis_weights[0] = 1.0f;
  }
  safe_divide = max(1, safe_divide);
  /***step03->综合障碍物权重计算***/
  for (int i = 0; i < obs_num; ++i) {
    //发生碰撞->只考虑碰撞障碍物
    if (safe_divide >= 1) {
      obstacles[i].weight = obs_dis_weights[i] / safe_divide;
    }
    //未发生碰撞->决断权重+距离权重
    else {
      obstacles[i].weight = weights(i) * obs_dis_weights[i];
    }
  }
}

/**
 * @brief:障碍物正交矩阵和特征向量矩阵计算
 * @param:
 *      agent:圆形机器单元
 * @return:agent对应的计算障碍物
 */
void DynamicSys::ComputeAgentsCalObsInfo() {
  /***step01->全部"计算障碍物"特征向量计算***/
  vector<port::CalculatedObs> all_calculated_obs;
  for (auto& agent : ab::g_agents) {
    // agents计算障碍物--基础矩阵，特征距离，避障方向计算
    CalAgentsCalObsBasicInfo(agent);
    all_calculated_obs.insert(all_calculated_obs.end(), agent.calculated_obstacles.begin(), agent.calculated_obstacles.end());
  }
  /***step02->全部"计算障碍物"各自权重计算***/
  CalObsWeights(all_calculated_obs);
  /***step03->各agents对应的“计算障碍物"分配***/
  auto it = all_calculated_obs.begin();
  int obs_num = ab::g_virtual_obs.size();
  for (int i = 0; i < ab::g_agents.size(); i++) {
    /***agent障碍匹配***/
    if (i < ab::g_agents.size() - 1) {
      ab::g_agents[i].calculated_obstacles.assign(it + i * obs_num, it + (i + 1) * obs_num);
    } else {
      ab::g_agents[i].calculated_obstacles.assign(it + i * obs_num, all_calculated_obs.end());
    }
  }
}

//内部决策左右绕障方向
void DynamicSys::CalStrechingMat(port::CircleAgent& agent) {
  // 特征值元素计算
  for (int i = 0; i < agent.calculated_obstacles.size(); i++) {
    float gamma_dis = agent.calculated_obstacles[i].gamma_dis;
    float lammda = powf(gamma_dis, 1.0 / REACT_FACTOR);
    // 引导向量特征值
    float eigenvalue_reference, eigenvalue_tangent;
    // 调整系数
    float repulsion_factor, tangent_factor, tangent_dis_factor, tangent_angle_factor;
    repulsion_factor = tangent_factor = tangent_dis_factor = tangent_angle_factor = 0.0f;
    // 绕障方向选择
    auto given_dir = agent.calculated_obstacles[i].avoid_dir;
    port::AvoidDir obs_avoid_dir;
    //[-TEST-]
    given_dir = port::AvoidDir::left;
    //车头方向与期望方向相反，强制约束期望方向(因为一个障碍物可能改变agent整体的期望速度方向)
    // agent.ref_vel_vec = LimitIntersectionAngle(ab::g_circle.heading_dir, agent.ref_vel_vec, 0.8*M_PI);
    // [x]:如果不指定绕障方向，则根据车头方向，设定绕障方向
    if (given_dir == port::AvoidDir::random) {
      //车头与障碍物切线方向计算
      float h_vs_t_angle = ab::g_circle.heading_dir.dot(agent.calculated_obstacles[i].tangent_vector);
      float angle_deg = acosf(h_vs_t_angle) * 180.0f / M_PI;
      if (angle_deg < 130.0f) {
        obs_avoid_dir = port::AvoidDir::left;
      } else {
        obs_avoid_dir = port::AvoidDir::right;
      }
    } else {
      obs_avoid_dir = given_dir;
    }
    int tangent_dir;
    float align = agent.ref_vel_vec.dot(agent.calculated_obstacles[i].tangent_vector);
    tangent_dir = fabs(align) / align;
    // tail effect方向判断系数
    float target_obs_angle_cos = agent.ref_vel_vec.normalized().dot(agent.calculated_obstacles[i].normal_vector);
    // 安全区域正常线性分解
    if (gamma_dis >= 1.f) {
      // tail effect约束
      //期望方向与障碍物法向趋同(期望背离障碍)
      if (target_obs_angle_cos > 0) {
        eigenvalue_reference = 1.0f;
        eigenvalue_tangent = 1.0f;
        tangent_dis_factor = tangent_angle_factor = 0.f;  // 绕过障碍物，不再增加切向分量
      }
      // 期望方向与障碍物法线相反(期望驶向障碍)
      else {
        eigenvalue_reference = 1 - 1.0f / lammda;
        tangent_dis_factor = 1 / powf(lammda, 3);
        tangent_angle_factor = powf(fabs(target_obs_angle_cos), 5);
        //绕障方向约束 //NOTE:方向引导系数与流场响应系数需要同步调整(一升一降)
        if (obs_avoid_dir == port::AvoidDir::left) {
          //压缩障碍左切线分量or拉伸障碍右切线分量(加强反向引导，防止矫正太晚,机器距离障碍物太近）
          eigenvalue_tangent = 1 + tangent_dir * (3.0f / lammda);
          eigenvalue_tangent = min(2.0f, eigenvalue_tangent);
          tangent_factor = 0.5f * tangent_dis_factor * tangent_angle_factor;  //左切线引导
        } else if (obs_avoid_dir == port::AvoidDir::right) {
          //压缩障碍右切线分量or拉伸障碍左切线分量(加强反向引导，防止矫正太晚,机器距离障碍物太近）
          eigenvalue_tangent = 1 - tangent_dir * (3.0f / lammda);
          eigenvalue_tangent = min(2.0f, eigenvalue_tangent);
          tangent_factor = (-1) * 0.5f * tangent_dis_factor * tangent_angle_factor;  //右切线引导
        }
      }
    }
    // 突破安全边界后的保护
    else {
      // tail effect约束
      // 期望方向与障碍物法向趋同(期望背离障碍)
      if (target_obs_angle_cos > 0) {
        eigenvalue_reference = 0.0f;
        tangent_dis_factor = tangent_angle_factor = 0.f;  // 绕过障碍物，不再增加切向分量
        eigenvalue_tangent = 1.0;
      }
      // 期望速度方向与障碍物方向相反(突破安全边界后仍然希望靠近障碍物)
      else {
        eigenvalue_reference = 0.0f;
        //绕障方向约束
        if (obs_avoid_dir == port::AvoidDir::left) {
          //车头沿障碍左侧(正值)->正向拉伸，车头沿障碍右侧(负值)->反向拉伸
          eigenvalue_tangent = tangent_dir * 3.0f;
          eigenvalue_tangent = min(2.0f, eigenvalue_tangent);
        } else if (obs_avoid_dir == port::AvoidDir::right) {
          //车头沿障碍左侧->反向拉伸，车头沿障碍右侧->正向拉伸
          eigenvalue_tangent = -tangent_dir * 3.0f;
          eigenvalue_tangent = min(2.0f, eigenvalue_tangent);
        }
      }
      repulsion_factor = powf(-lammda + 1.f, 2);
      tangent_factor = 0.0f;
      // note:reference and tangent分解有时会出问题 //!!!:留意
      agent.calculated_obstacles[i].reference_vector = agent.calculated_obstacles[i].normal_vector;
      if (gamma_dis < 0.05f) {
        cout << "bug here!!!" << endl;
      }
    }
    repulsion_factor /= ab::g_agents.size();
    tangent_factor /= ab::g_agents.size();
    Eigen::DiagonalMatrix<float, 2> diagonal_eigenvalues(eigenvalue_reference, eigenvalue_tangent);
    agent.calculated_obstacles[i].streching_matrix = diagonal_eigenvalues;
    agent.calculated_obstacles[i].trim_vector =
        repulsion_factor * agent.calculated_obstacles[i].normal_vector + tangent_factor * agent.calculated_obstacles[i].tangent_vector;
    // 对障碍物reference方向进行约束([车头]朝向切向为基准)
    vec2f benchmark_vec = tangent_dir * agent.calculated_obstacles[i].tangent_vector;
    agent.calculated_obstacles[i].reference_vector =
        LimitIntersectionAngle(benchmark_vec, agent.calculated_obstacles[i].reference_vector, 0.4 * M_PI);
    agent.calculated_obstacles[i].basic_matrix << agent.calculated_obstacles[i].reference_vector, benchmark_vec;
  }
}

/**
 * @breif:agent对应modulated速度计算(速度方向合成)
 * @param:
 *      agent:圆形机器单元(note:更新agent对应的各计算障碍物对应的modulate v)
 */
void DynamicSys::CalAgentModulatedV(port::CircleAgent& agent) {
  // 合成速度magnitude计算
  float vel_magnitude = 0.f;
  // 原始速度坐标系计算
  vec2f oriV_normal = agent.cur_vel_vec.normalized();
  vec2f oriV_tangent(-1 * oriV_normal(1), oriV_normal(0));
  mat2f oriV_frame;
  oriV_frame << oriV_normal, oriV_tangent;
  mat2f to_oriV_frame = oriV_frame.transpose();
  float sum_angle_space = 0.f;
  // 障碍物对应的modulated矩阵计算
  for (int i = 0; i < agent.calculated_obstacles.size(); i++) {
    port::CalculatedObs each_obs = agent.calculated_obstacles[i];
    mat2f modulate_matrix = each_obs.basic_matrix * each_obs.streching_matrix * each_obs.basic_matrix.inverse();
    // mat2f modulate_matrix = each_obs.orthogonal_matrix * each_obs.streching_matrix * each_obs.orthogonal_matrix.inverse();
    vec2f modulate_v = modulate_matrix * agent.ref_vel_vec + agent.calculated_obstacles[i].trim_vector;
    agent.calculated_obstacles[i].modulate_vector = modulate_v;
    // 速度大小合成
    vel_magnitude += modulate_v.norm() * agent.calculated_obstacles[i].weight;
    // 统一到当前速度坐标系
    vec2f Ki_normal = (to_oriV_frame * modulate_v).normalized();
    // 机器相对障碍物运动趋势计算(方向夹角)
    float angle_space = atan2f(Ki_normal(1), Ki_normal(0));
    sum_angle_space += angle_space * agent.calculated_obstacles[i].weight;
  }
  // 障碍物合成速度矢量计算(当前速度坐标系)
  vec2f reconstruct_vel;
  if (fabsf(sum_angle_space) < 1E-2) {
    reconstruct_vel << 1, 0;
  } else {
    reconstruct_vel << cos(sum_angle_space), sin(sum_angle_space);
  }
  // 还原到全局坐标系，初步获得agent modulate速度方向
  agent.compound_vel = (oriV_frame * reconstruct_vel).normalized();
}

/**
 * @brief:机器agent调制速度合成
 * @return:合成速度
 */
port::Twist DynamicSys::SynthesizeAvoidCmd(const float& planning_v) {
  // 速度模长计算
  float agent_vel_norm = planning_v / ab::g_agents.size();
  // agents速度合成
  mat2f rotate_to_local;
  rotate_to_local << cos(ab::g_robot_pose.theta), sin(ab::g_robot_pose.theta), -sin(ab::g_robot_pose.theta), cos(ab::g_robot_pose.theta);
  Eigen::Vector2f synthesis_cmd_vel(0.0f, 0.0f);
  for (auto& agent : ab::g_agents) {
    // 计算agent对应的modulate速度(方向+模长)
    agent.compound_vel = agent_vel_norm * agent.compound_vel;
    // 速度agent速度合成(全局速度矢量转局部控制速度[v,w])
    synthesis_cmd_vel += agent.jacobian.inverse() * rotate_to_local * agent.compound_vel;
  }
  // synthesis_cmd_vel += 2 * ab::g_agents[1].jacobian.inverse() * rotate_to_local * ab::g_agents[1].compound_vel;
  auto cmd_vel = port::Twist(synthesis_cmd_vel(0), synthesis_cmd_vel(1));
  return cmd_vel;
}

/**
 * @breif:动态系统功能函数
 */
port::Twist DynamicSys::DynamicSysAvoid(const float& planning_v) {
  /***step01->agents对应的“计算障碍物”变量计算***/
  ComputeAgentsCalObsInfo();
  /***step02->各agents modulated速度计算***/
  for (auto& agent : ab::g_agents) {
    /***特征向量矩阵***/
    CalStrechingMat(agent);
    /***障碍物Modulate速度方向合成***/
    CalAgentModulatedV(agent);
  }
  /***step03->最终控制速度合成***/
  auto cmd = SynthesizeAvoidCmd(planning_v);
  return cmd;
}

}  // namespace algorithm
}  // namespace control
}  // namespace modules