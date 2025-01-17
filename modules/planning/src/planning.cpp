#include "planning.h"

#include <random>

#include "tools/math_tools.h"

#include "basic_algorithm/interpolation.hpp"

namespace mathTools = utilities::mathTools;

namespace modules {
namespace planning {

utilities::basicAlg::InterpolateTools interpolator;

vector<port::CommonPose> PathPlanner::PlanningMultiTarget(const port::CommonPose& start_pose, const float& dis_offset) {
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<double> random_dis(-dis_offset, dis_offset);
  std::uniform_real_distribution<double> random_yaw(-M_PI, M_PI);
  vector<float> xs(2);
  vector<float> ys(2);
  xs[0] = start_pose.x + random_dis(gen);
  ys[0] = start_pose.y + random_dis(gen);
  // NOTE:刚体agent中心距离(机身纵轴)
  xs[1] = xs[0] - sin(random_yaw(gen)) * 0.38f;
  ys[1] = ys[0] - cos(random_yaw(gen)) * 0.38f;
  // 生成多目标点
  vector<port::CommonPose> planning_path(2);
  for (int i = 0; i < 2; i++) {
    planning_path[i].x = xs[i];
    planning_path[i].y = ys[i];
  }
  return planning_path;
}

vector<port::CommonPose> PathPlanner::PlanningCurvePath(const port::CommonPose& start_pose, const int points_num, const float& start_offset,
                                                        const float& path_offset) {
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<double> path_dis(-path_offset, path_offset);
  std::uniform_real_distribution<double> start_dis(-start_offset, start_offset);
  vector<float> xs(points_num);
  vector<float> ys(points_num);
  xs[0] = start_pose.x + start_dis(gen);
  ys[0] = start_pose.y + start_dis(gen);
  for (int i = 1; i < points_num; ++i) {
    xs[i] = xs[i - 1] + path_dis(gen);
    ys[i] = ys[i - 1] + path_dis(gen);
  }
  // 样条插值得到平滑曲线
  auto [xs2, ys2] = interpolator.InterpolateXY(xs, ys, resolution_);
  // 生成随机路径
  int path_size = xs2.size();
  vector<port::CommonPose> planning_path(path_size);
  for (int i = 0; i < path_size; i++) {
    planning_path[i].x = xs2[i];
    planning_path[i].y = ys2[i];
  }
  return planning_path;
}

vector<port::CommonPose> PathPlanner::PlanningStraightPath(const port::CommonPose& start_pose, const float& start_offset, const float& path_offset) {
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<double> path_dis(-path_offset, path_offset);
  std::uniform_real_distribution<double> start_dis(-start_offset, start_offset);
  vector<float> xs(2), ys(2);
  // xs[0] = start_pose.x+start_dis(gen);
  // ys[0] = start_pose.y+start_dis(gen);
  // xs[1] = start_pose.x+path_dis(gen);
  // ys[1] = start_pose.y+path_dis(gen);
  xs[0] = start_pose.x;
  ys[0] = start_pose.y;
  xs[1] = start_pose.x + path_offset;
  ys[1] = start_pose.y;
  // 样条插值得到平滑曲线
  auto [xs2, ys2] = interpolator.LineInterpolate(xs, ys, resolution_);
  // 生成随机路径
  int path_size = xs2.size();
  vector<port::CommonPose> planning_path(path_size);
  for (int i = 0; i < path_size; i++) {
    planning_path[i].x = xs2[i];
    planning_path[i].y = ys2[i];
  }
  return planning_path;
}

vector<port::CommonPose> PathPlanner::PlanningEllipPath(const port::CommonPose& start_pose, const float& a, const float& b) {
  //椭圆周长
  float length = 2 * M_PI * b + 4 * fabs(a - b);
  int points_num = length / resolution_;
  vector<port::CommonPose> planning_path(points_num);
  boundary_.clear();
  float test_dis = 2.2;
  // use a linespace to have a full rotation angle between [-pi, pi]
  vector<float> alpha = mathTools::linspace(M_PI, -M_PI, points_num);
  //椭圆形轮廓点集
  for (unsigned int i = 0; i < points_num; ++i) {
    planning_path[i].x = a * sin(alpha[i]) + start_pose.x;
    planning_path[i].y = b * cos(alpha[i]) + start_pose.y;
    //边界削减点数(方便后面障碍物边界内筛选)
    if (i % 5 == 0 && i != 0) {
      Eigen::Vector2f boundary_point;
      boundary_point(0) = (a + test_dis) * sin(alpha[i]) + start_pose.x;
      boundary_point(1) = (b + test_dis) * cos(alpha[i]) + start_pose.y;
      boundary_.push_back(boundary_point);
    }
  }
  return planning_path;
}

vector<Eigen::Vector2f> PathPlanner::GetBoundary() { return boundary_; }

vector<pair<vector<float>, vector<float>>> PathPlanner::BoundaryPlotObs() {
  vector<pair<vector<float>, vector<float>>> test(1);
  int num = boundary_.size();
  vector<float> x(num), y(num);
  for (int i = 0; i < num; i++) {
    x[i] = boundary_[i](0);
    y[i] = boundary_[i](1);
  }
  test[0] = make_pair(x, y);
  return test;
}

}  // namespace planning
}  // namespace modules