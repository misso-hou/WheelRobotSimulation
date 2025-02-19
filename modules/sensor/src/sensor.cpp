#include "sensor.h"

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <opencv2/opencv.hpp>

#include "robot_configuration.h"
#include "tools/math_tools.h"

namespace modules {
namespace sensor {

using namespace modules;
namespace mathTools = utilities::mathTools;
// boost几何计算库
namespace bg = boost::geometry;
typedef bg::model::d2::point_xy<float> Point;
typedef bg::model::polygon<Point> Polygon;
typedef bg::model::segment<Point> Segment;
typedef bg::model::linestring<Point> LineString;

LidarSensor::LidarSensor(float res, float range) {
  range_ = range;
  resolution_ = res * M_PI / 180.0f;  //弧度
  sector_num_ = 360.f / res;
  //雷达数据
  lidar_data_.resize(2);
  lidar_data_[0].resize(sector_num_);
  lidar_data_[1].resize(sector_num_);
}

/**
 * @brief: 功能描述：获取雷达数据(全部射线数据)
 */
vector<vector<float>> LidarSensor::GetLidarData() { return lidar_data_; }

/**
 * @brief: 功能描述：障碍物点云雷达射线分区
 * @params:
 *	  robot_pose: 机器当前定位
 *	  env_obs: 全局环境中的障碍物
 * @return: 射线分组内的障碍物点云
 */
vector<vector<float>> LidarSensor::ObsDetectByReflect(const port::CommonPose& sensor_pose,
                                                      const vector<pair<vector<float>, vector<float>>>& env_obs) {
  //障碍物轮廓提取
  vector<Polygon> obs_polygon(env_obs.size());
  for (uint i = 0; i < env_obs.size(); i++) {
    //(注意:障碍物顶点首尾相接，序号为0-1-2-3-0)
    for (uint j = 0; j < env_obs[i].first.size(); j++) {
      float point_x, point_y;
      point_x = env_obs[i].first[j];
      point_y = env_obs[i].second[j];
      bg::append(obs_polygon[i], Point(point_x, point_y));
    }
  }
  //雷达仿真可视化
  vector<vector<float>> obs_data(2);
  //雷达射线遍历
  for (uint i = 0; i < sector_num_; i++) {
    /*step01->定义射线*/
    Point ray_origin(sensor_pose.x, sensor_pose.y);  //射线起点
    float ray_angle = i * resolution_;
    ray_angle = mathTools::NormalizeAngle(ray_angle + sensor_pose.theta);
    float ray_end_x = sensor_pose.x + range_ * cos(ray_angle);
    float ray_end_y = sensor_pose.y + range_ * sin(ray_angle);
    Point ray_end(ray_end_x, ray_end_y);  //射线方向
    LineString ray;
    ray.push_back(ray_origin);
    ray.push_back(ray_end);
    /*step02->遍历障碍物->计算所有交点*/
    vector<Point> intersections;
    for (auto polygon : obs_polygon) {
      bg::intersection(ray, polygon, intersections);
    }
    /*step03->筛选最近交点*/
    if (intersections.empty()) {
      lidar_data_[0][i] = ray_end_x;
      lidar_data_[1][i] = ray_end_y;
    } else {
      //筛选最近点
      float min_dist = range_;
      for (auto point : intersections) {
        float dist = bg::distance(ray_origin, point);
        if (dist < min_dist) {
          min_dist = dist;
          lidar_data_[0][i] = bg::get<0>(point);
          lidar_data_[1][i] = bg::get<1>(point);
        }
      }
      //转到局部坐标系
      port::CommonPose g_obs_pose(lidar_data_[0][i], lidar_data_[1][i], 0.f);
      auto l_obs_pose = mathTools::Global2Rob(sensor_pose, g_obs_pose);
      obs_data[0].push_back(l_obs_pose.x);
      obs_data[1].push_back(l_obs_pose.y);
    }
  }
  return obs_data;
}

/**
 * @brief:功能描述：雷达射线障碍物点云识别(角度分区识别方式)
 * @params:
 *	  robot_pose:机器当前定位
 *	  env_obs:全局环境中的障碍物
 * @return:模拟雷达识别到的障碍物点云((X1，X2,...)，(y1,y2,...))
 */
vector<vector<float>> LidarSensor::ObsDetectByAngle(const port::CommonPose& sensor_pose, const vector<pair<vector<float>, vector<float>>>& env_obs) {
  //角度区间分组
  vector<vector<port::SamplePoint>> subgroup(sector_num_);
  /***按角度分组***/
  for (auto one_obs : env_obs) {
    vector<float> one_obs_x, one_obs_y;
    one_obs_x = one_obs.first;
    one_obs_y = one_obs.second;
    for (int i = 0; i < one_obs_x.size(); i++) {
      //全局转局部
      port::CommonPose point_pose(one_obs_x[i], one_obs_y[i], 0.0f);
      auto local_pose = mathTools::Global2Rob(sensor_pose, point_pose);
      //过滤传感器范围外的点
      if (local_pose.x > vehicle::front_vision_boundary[2][0] || local_pose.x < vehicle::back_vision_boundary[1][0] ||
          local_pose.y > vehicle::left_vision_boundary[2][1] || local_pose.y < vehicle::right_vision_boundary[1][1]) {
        continue;
      }
      port::vec2f point(local_pose.x, local_pose.y);
      port::vec2f pose(sensor_pose.x, sensor_pose.y);
      //中心->点向量
      float polar_r = point.norm();  // 极坐标半径
      // 极坐标方位角计算
      float polar_theta = atan2f(point(1), point(0));
      if (polar_theta < 0) {
        polar_theta += 2 * M_PI;
      }
      int subgroup_index = (int)(polar_theta / resolution_);
      subgroup_index = min(subgroup_index, sector_num_ - 1);
      port::SamplePoint new_point;
      new_point.pose = point;
      new_point.polar_dis = polar_r;
      new_point.phase = (int)(polar_theta / (M_PI / 2)) + 1;
      new_point.angle = polar_theta * 180 / M_PI;
      // 插入新的采样点
      subgroup[subgroup_index].push_back(new_point);
    }
  }
  /***计算各角度区间内最近的障碍物点***/
  vector<float> x_arr, y_arr;
  //分组遍历(每组内选出一个最近点)
  for (int i = 0; i < sector_num_; i++) {
    if (subgroup[i].empty()) continue;
    float min_dis = 100.0;  //一个超过传感器测量范围的值
    port::SamplePoint sub_sample_point;
    sub_sample_point.pose << 0.f, 0.f;
    //寻找组内最近点
    for (auto subgroup_point : subgroup[i]) {
      //过滤大于采样距离外的点,且角度区间内只保留最近点
      if (subgroup_point.polar_dis < min_dis) {
        min_dis = subgroup_point.polar_dis;
        sub_sample_point = subgroup_point;
      }
    }
    // 过滤空值
    if (sub_sample_point.pose.norm() > 0.f) {
      x_arr.push_back(sub_sample_point.pose(0));
      y_arr.push_back(sub_sample_point.pose(1));
    }
  }
  vector<vector<float>> detected_obs(2);
  detected_obs[0] = x_arr;
  detected_obs[1] = y_arr;
  return detected_obs;
}

/**
 * @brief: 功能描述：雷达传感器数据更新
 * @params:
 *      robot_pose:机器当前定位
 *      env_obs:全局环境中的障碍物
 * @return: 模拟雷达识别到的障碍物点云
 */
vector<Eigen::Vector2f> LidarSensor::UpdateAiSensor(const port::CommonPose& robot_pose, const vector<pair<vector<float>, vector<float>>>& env_obs) {
  // ai vision轮廓(全局坐标[障碍物在全局坐标系])
  vector<vector<cv::Point2f>> polygon_points(4);
  for (auto point : vehicle::front_vision_boundary) {
    polygon_points[0].push_back(cv::Point2f(point[0], point[1]));
  }
  for (auto point : vehicle::left_vision_boundary) {
    polygon_points[1].push_back(cv::Point2f(point[0], point[1]));
  }
  for (auto point : vehicle::right_vision_boundary) {
    polygon_points[2].push_back(cv::Point2f(point[0], point[1]));
  }
  for (auto point : vehicle::back_vision_boundary) {
    polygon_points[3].push_back(cv::Point2f(point[0], point[1]));
  }
  //传感器识别障碍物获取
  vector<float> obs_x, obs_y;
  // auto detected_obs = ObsDetectByAngle(robot_pose, env_obs);
  // TODO:测试Lidar射线数据
  auto detected_obs = ObsDetectByReflect(robot_pose, env_obs);
  obs_x = detected_obs[0];
  obs_y = detected_obs[1];
  // 障碍物点处理->平移到局部坐标系+判断是否在ai vision范围内
  vector<Eigen::Vector2f> sim_ai_obs;  // 虚拟ai障碍物数据
  for (int i = 0; i < obs_x.size(); i++) {
    cv::Point2f one_point(obs_x[i], obs_y[i]);
    //判断是否在视野内部
    for (int j = 0; j < polygon_points.size(); j++) {
      //虚拟ai障碍物点转局部坐标系
      double result = cv::pointPolygonTest(polygon_points[j], one_point, false);
      if (result > 0) {
        //虚拟ai障碍物点转局部坐标系
        Eigen::Vector2f temp_ai_point;
        temp_ai_point(0) = one_point.x;
        temp_ai_point(1) = one_point.y;
        sim_ai_obs.push_back(temp_ai_point);
      }
    }
  }
  return sim_ai_obs;
}

}  // namespace sensor
}  // namespace modules