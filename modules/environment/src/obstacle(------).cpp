
#include "obstacle.h"

#include <random>

#include "tools/math_tools.h"

using namespace std;
namespace mathTools = utilities::mathTools;

namespace modules {
namespace env {

Obstacle::Obstacle(ObsType type, const port::CommonPose& pose, const port::Twist& vel, float dt) {
  obs_type_ = type;
  center_pose_ = pose;
  obs_vel_ = vel;
  dt_ = dt;
}

void Obstacle::SetObsProperty(ObsType type, DataType dtype, const port::CommonPose& pose, const port::Twist& vel, const float& dt, const float& dev) {
  obs_type_ = type;
  data_type_ = dtype;
  center_pose_ = pose;
  obs_vel_ = vel;
  dt_ = dt;
  sdev_ = dev;
}

void Obstacle::SetEllipsoidAxis(const Eigen::Array2f& axis_length, int points_num, const float& ds) {
  axis_length_ = axis_length;
  if (fabs(ds) < 0.00001) {
    points_num_ = points_num;
  } else {
    points_num_ = M_PI * axis_length_(0) * axis_length_(1) / ds;
  }
}

void Obstacle::SetRandomObsParam(const float& radius, int edge_num, int points_num) {
  radius_ = radius;
  edge_num_ = edge_num;
  points_num_ = points_num;
}

void Obstacle::SetRectangleObsParam(const float& height, const float& width, const float& ds) {
  height_ = height;
  width_ = width;
  ds_ = ds;
  points_num_ = height_ * width_ / ds;
}

void Obstacle::ObsInit() {
  if (data_type_ == DataType::COUNTOUR) {
    ContourInitialize();
  } else if (data_type_ == DataType::COUNTOURCLOUD) {
    CloudRandomObsInit();
  }
}

void Obstacle::CloudRandomObsInit() {
  time_t t;
  vector<float> amps, phases;
  float R;
  srand((unsigned)time(&t));
  srand((unsigned)rand());

  for (int i = 0; i < edge_num_; i++) {
    float s = (rand() / (float)RAND_MAX);
    amps.push_back(1.0 / (2.0 * edge_num_) * s * 4);  // 最后一个系数调节凹凸程度
    phases.push_back(s * 2.0 * M_PI);
  }
  // 障碍物边界点计算
  vector<float> alpha = mathTools::linspace(-M_PI, M_PI, points_num_);
  // 轮廓点集
  random_r_.resize(points_num_);
  for (unsigned int i = 0; i < points_num_; ++i) {
    random_r_[i] = radius_;
    for (int j = 0; j < edge_num_ - 1; j++) {
      random_r_[i] += (amps[j] * cos((j + 1) * alpha[i] + phases[j]));
    }
  }
}

void Obstacle::ContourInitialize() {
  // 根据障碍物类型，生成原始障碍物边界点云(局部坐标系)
  switch (obs_type_) {
    case ObsType::ELLIP: {
      GenerateEllisoidObs();
      break;
    }
    case ObsType::RANDOM: {
      GenerateRandomShapeObs();
      break;
    }
    case ObsType::Rectangle: {
      GenerateRectangleVertexObs();
    }
    default:
      break;
  }
  // 静态障碍物，点云只计算一次
  if (fabs(dt_ - 0.f) < 0.0001) {
    // 显示障碍物与外发计算数据格式
    vector<float> obs_x(origin_obs_points_.rows()), obs_y(origin_obs_points_.rows());
    trans_obs_points_.clear();
    Eigen::Vector2f pose(center_pose_.x, center_pose_.y);
    Eigen::Matrix2f rotate_matrix;  // local to global
    rotate_matrix << cos(center_pose_.theta), -sin(center_pose_.theta), sin(center_pose_.theta), cos(center_pose_.theta);
    for (int i = 0; i < origin_obs_points_.rows(); i++) {
      Eigen::Vector2f local_point(origin_obs_points_.row(i)(0), origin_obs_points_.row(i)(1));
      Eigen::Vector2f global_point = rotate_matrix * local_point + pose;
      trans_obs_points_.push_back(global_point);
      obs_x[i] = global_point(0);
      obs_y[i] = global_point(1);
    }
    obs_show_ = make_pair(obs_x, obs_y);
  }
}

void Obstacle::CloudObsUpdate() {
  // 根据障碍物类型，生成原始障碍物边界点云(局部坐标系)
  switch (obs_type_) {
    case ObsType::ELLIP: {
      EllipCloudObsUpdate();
      break;
    }
    case ObsType::RANDOM: {
      RandomCloudObsUpdate();
      break;
    }
    case ObsType::Rectangle: {
      RectCloudObsUpdate();
    }
    default:
      break;
  }
  // step01->更新障碍物中心位姿
  UpdateObsState();
  // step02->障碍物点云更新
  ObsTransform();
}

void Obstacle::RandomCloudObsUpdate() {
  // 设置随机数生成器
  std::random_device rd;
  std::mt19937 gen(rd());
  // 轮廓内随机点生成
  Eigen::MatrixXf obs_points(3 * points_num_, 2);
  // 障碍物边界点计算
  vector<float> alpha = mathTools::linspace(-M_PI, M_PI, points_num_);
  // 椭圆形轮廓点集
  for (unsigned int i = 0; i < points_num_; ++i) {
    for (unsigned int j = 0; j < 3; ++j) {
      std::uniform_real_distribution<> angle_dis(-0.1, 0.1);
      float local_angle = alpha[i] + angle_dis(gen);
      //随机半径比例
      std::normal_distribution<float> ratio_dis(1.0f, 0.05);
      float radius_ratio = ratio_dis(gen) > 1.0f ? 1.0f : ratio_dis(gen);
      // 计算椭圆内的随机点坐标+正态分布噪声
      std::normal_distribution<float> noise_x(0.0f, sdev_);
      std::normal_distribution<float> noise_y(0.0f, sdev_);
      obs_points(3 * i + j, 0) = random_r_[i] * radius_ratio * cos(local_angle) + noise_x(gen);
      obs_points(3 * i + j, 1) = random_r_[i] * radius_ratio * sin(local_angle) + noise_y(gen);
    }
  }
  origin_obs_points_ = obs_points;
}

void Obstacle::EllipCloudObsUpdate() {
  // 设置随机数生成器
  std::random_device rd;
  std::mt19937 gen(rd());
  // 轮廓内随机点生成
  Eigen::MatrixXf obs_points(points_num_, 2);
  // 椭圆形轮廓点集
  for (unsigned int i = 0; i < points_num_; ++i) {
    // 随机角度，均匀分布在(0,2*PI)范围内
    std::uniform_real_distribution<> angle_dis(0.0, 2 * M_PI);
    float angle = angle_dis(gen);
    // 随机生成半径比例，确保生成的点在椭圆内
    // 使用椭圆参数a和b，通过极坐标转换随机点
    std::normal_distribution<float> ratio_dis(1.0f, 0.03);
    float radius_ratio = ratio_dis(gen) > 1.0f ? 1.0f : ratio_dis(gen);
    // 计算椭圆内的随机点坐标+正态分布噪声
    std::normal_distribution<float> noise_x(0.0f, sdev_);
    std::normal_distribution<float> noise_y(0.0f, sdev_);
    obs_points(i, 0) = axis_length_(0) * radius_ratio * cos(angle) + noise_x(gen);
    obs_points(i, 1) = axis_length_(1) * radius_ratio * sin(angle) + noise_y(gen);
  }
  origin_obs_points_ = obs_points;
}

// 随机点云障碍物更新
void Obstacle::RectCloudObsUpdate() {
  // 设置随机数生成器
  std::random_device rd;
  std::mt19937 gen(rd());
  // 矩形顶点
  float halfWidth = width_ / 2.0;
  float halfHeight = height_ / 2.0;
  // 轮廓内随机点生成
  Eigen::MatrixXf obs_points(points_num_, 2);
  // 椭圆形轮廓点集
  for (unsigned int i = 0; i < points_num_; ++i) {
    std::uniform_real_distribution<> point_x(-halfWidth, halfWidth);
    std::uniform_real_distribution<> point_y(-halfHeight, halfHeight);
    //正态分布噪声
    std::normal_distribution<float> noise_x(0.0f, sdev_);
    std::normal_distribution<float> noise_y(0.0f, sdev_);
    obs_points(i, 0) = point_x(gen) + noise_x(gen);
    obs_points(i, 1) = point_y(gen) + noise_y(gen);
  }
  origin_obs_points_ = obs_points;
}

/*更新障碍物信息*/
void Obstacle::CoutourObsUpdate() {
  //静态障碍物无需更新
  if (fabs(dt_ - 0.f) < 0.0001) {
    return;
  }
  // step01->更新障碍物位姿
  UpdateObsState();
  // step02->障碍物点云更新
  ObsTransform();
}

void Obstacle::GenerateEllisoidObs() {
  // use a linespace to have a full rotation angle betweent [-pi, pi]
  vector<float> alpha = mathTools::linspace(-M_PI, M_PI, points_num_);
  Eigen::MatrixXf obs_points(points_num_, 2);
  //椭圆形轮廓点集
  for (unsigned int i = 0; i < points_num_; ++i) {
    obs_points(i, 0) = axis_length_(0) * sin(alpha[i]);
    obs_points(i, 1) = axis_length_(1) * cos(alpha[i]);
  }
  origin_obs_points_ = obs_points;
}

void Obstacle::GenerateRectangleVertexObs() {
  float halfWidth = width_ / 2.0;
  float halfHeight = height_ / 2.0;
  //矩形顶点
  Eigen::MatrixXf obs_points(5, 2);
  obs_points << halfWidth, halfHeight, -halfWidth, halfHeight, -halfWidth, -halfHeight, halfWidth, -halfHeight, halfWidth, halfHeight;
  origin_obs_points_ = obs_points;
}

void Obstacle::GenerateRandomShapeObs() {
  time_t t;
  vector<float> amps, phases;
  float R;
  srand((unsigned)time(&t));
  srand((unsigned)rand());
  for (int i = 0; i < edge_num_; i++) {
    float s = (rand() / (float)RAND_MAX);
    amps.push_back(1.0 / (2.0 * edge_num_) * s * 4);  // 最后一个系数调节凹凸程度
    phases.push_back(s * 2.0 * M_PI);
  }
  // 障碍物边界点计算
  vector<float> alpha = mathTools::linspace(-M_PI, M_PI, points_num_);
  Eigen::MatrixXf obs_points(points_num_, 2);
  // 椭圆形轮廓点集
  for (unsigned int i = 0; i < points_num_; ++i) {
    R = radius_;
    for (int j = 0; j < edge_num_ - 1; j++) {
      R = R + (amps[j] * cos((j + 1) * alpha[i] + phases[j]));
    }
    obs_points(i, 0) = cos(alpha[i]) * R;
    obs_points(i, 1) = sin(alpha[i]) * R;
  }
  origin_obs_points_ = obs_points;
}

/*更新障碍物运动状态*/
void Obstacle::UpdateObsState() {
  center_pose_.x += dt_ * obs_vel_.linear * sin(center_pose_.theta);
  center_pose_.y += dt_ * obs_vel_.linear * cos(center_pose_.theta);
  center_pose_.theta += dt_ * obs_vel_.angular;
}

// 障碍物点坐标系变换
void Obstacle::ObsTransform() {
  // 坐标系转换矩阵
  Eigen::Matrix<float, 2, 3> transformationMatrix;
  transformationMatrix << cos(center_pose_.theta), -sin(center_pose_.theta), center_pose_.x, sin(center_pose_.theta), cos(center_pose_.theta),
      center_pose_.y;
  // 轮廓点矩阵变形
  Eigen::MatrixXf pointsWithOnes(origin_obs_points_.rows(), origin_obs_points_.cols() + 1);
  pointsWithOnes << origin_obs_points_, Eigen::MatrixXf::Ones(origin_obs_points_.rows(), 1);
  // 坐标系变换(local to global)
  Eigen::MatrixXf globalPoints = pointsWithOnes * transformationMatrix.transpose();
  // 更新障碍物轮廓
  obs_points_ = globalPoints;
  // 显示障碍物与外发计算数据格式
  vector<float> obs_x(obs_points_.rows()), obs_y(obs_points_.rows());
  trans_obs_points_.clear();
  for (int i = 0; i < obs_points_.rows(); i++) {
    trans_obs_points_.push_back(obs_points_.row(i));
    obs_x[i] = obs_points_(i, 0);
    obs_y[i] = obs_points_(i, 1);
  }
  obs_show_ = make_pair(obs_x, obs_y);
}

//障碍物更新
void Obstacle::ObsUpdate() {
  if (data_type_ == DataType::COUNTOUR) {
    CoutourObsUpdate();
  } else if (data_type_ == DataType::COUNTOURCLOUD) {
    CloudObsUpdate();
  }
}

}  // namespace env
}  // namespace modules
}  // namespace modules
