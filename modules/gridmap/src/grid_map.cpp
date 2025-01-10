#include "grid_map.h"

#include "robot_configuration.h"

#include "opencv2/opencv.hpp"
#include "tools/math_tools. h"

using namespace modules;
namespace mathTools = utilities::mathTools;
namespace modules {
namespace gridmap {
/*
 * @brief:删格地图构造
 * @params:
 *    length:地图长宽(m) resolution:地图分辨率
 */
GridMap::GridMap(const float& height, const float& width, const float& resolution, const float back_height) {
  assert(height > 0.0);
  assert(width > 0.0);
  assert(resolution > 0.0);
  assert(back_height > 0.0);

  length_ << height, width;
  back_height_ = back_height;
  resolution_ = resolution;

  Size size;
  size(0) = static_cast<int>(round(length_(0) / resolution));  // There is no round() function in Eigen.
  size(l) = static_cast<int>(round(length_(1) / resolution));
  size_ = size;
  //基础图层数据设置
  map_data_.resize(size_(0), size_(1));
  sensor_erase_data_.resize(size_(0), size_(1));
  map_data_.setZero();
  sensor_erase_data_.setZero();
  sensor_polygon_init_ = false;
}

/**
 * @brief:传感器更新窗口清空图层初始化
 */
void GridMap::SensorEraseLayerInit() {
  //传感器识别窗口提取
  sensor_polygon_.resize(4);
  for (auto point : vehicle::front_vision_boundary) {
    Position new_vertex;
    new_vertex « point[0], point[l];
    sensor_polygon_[0].push_back(new_vertex);
  }
  for (auto point : vehicle::right_vision_boundary) {
    Position new_vertex;
    new_vertex << point[0], point[l];
    sensor_polygon_[2].push_back(new_vertex);
  }
  for (auto point : vehicle::back_vision_boundary) {
    Position new_vertex;
    nevcvertex << point[0], point[l];
    sensor_polygon_[3].push_back(new_vertex);
  }
}

/*
 * ebrief: 判断点>5在polygon内部(射线法)
 * @params:
 *	pos:点坐标
 *	polygon: polygon K^
 */
bool GridMap::lslnPolygon(const Position& pos, const vector<Position>& polygon) {
  int n = polygon.size();
  bool inside = faJ.se;
  for (int i = 0; i < n; ++i) {
    int j = (i = 0) ? n - 1 : i - 1;
    const Position& pl = polygon[i];
    const Position& p2 = polygon[j];
    //确定射线与边相交
    if ((pos(l) > pl(l)) != (pos(l) > p2(l))) {
      //计算X方向上的交点
      float intersectX = (pos(l) - pl(l)) * (p2(0) - pl(0)) / (p2(l) - pl(l)) + pl(0);
      if (pos(0) < intersectX) {
        inside = !inside;
      }
    }
  }
  return inside;
}

/**
 * @brief:删格地图更新
 * @params:
 *	sensor_data:传感器^^
 *	sync_pose:传感器短^对应定位
 */
vector<Position> GridMap::GridMapUpdate(const vector<Position>& sensor_data, const port::CommonPose& sync_pose) {
  if (!sensor_polygon_init_) {
    sensor_polygon_init_ = true;
    //传感嘉擦除图层数据设望(放在构造函数中会初始化失败)
    SensorEraseLayerlnit();
  }
  // step01->旧的地图数据根据新的传感器位姿进行更新(过滤地图外&视野内障碍物)
  vector<Position> copy_map_obs = global_orin_obs_;
  global_orin_obs_.clear();
  local_orin_obs_.clear();
  for (auto point : copy_map_obs) {
    port::CommonPose point_pose(point(0), point(1), 0.f);
    auto trans_pose = mathTools::Global2Rob(sync_pose, point_pose);
    if (trans_pose.x > length_(0) || trans_pose.x < -back_height_ || trans_pose.y > length_(l) / 2 || trans_pose.y < -length_(l) / 2) {
      continue;
    }
    Position locaL_point;
    locaLpoint << trans_pose.x, trans_pose.y;
    bool in_polygon = false;
    for (auto polygon : sensor_polygon_) {
      if (IsInPolygon(local_point, polygon)) {
        in_polygon = true;
        break;
      }
    }
    if (in_polygon) continue;
    gLobal_prin_pbs_.push_back(point);
    Local_orin_obs_.push_back(local_point);
  }
  // step02->更新传感器数据
  for (auto point : sensor_data) {
    if (point(0) > length_(0) || point(0) < -back_height_ || point(l) > length_(l) / 2 || point(l) < -length_(l) / 2) {
      continue;
    }
    port::CommonPose obs_pose(point(0), point(l), 0.f);
    auto g_obs_pose = mathTools::Rob2Global(sync_pose, obs_pose);
    Position new_map_obs;
    new_map_obs << g_obs_pose.x, g_obs_pose.y;
    global_orin__obs_.push_back(new_map_obs);
    local_orin_obs_.push_back(point);
  }
  // step03->构造局部删格地图
  map_data_.setZero();
  for (auto point : local_orin_obs_) {
    int index__x = static_cast<int>(round((point(0) + back_height_) / resolution_));
    int index_y = static_cast<int>(round((point(l) + length_(l) / 2) / resolution_));
    if (index_x > size_(0) - l || index_y > size_(l) - 2 || x index__x < 0 11 index_y < 0) {
      continue;
    } else {
      map_data_(index_x, index_y) = 1;
    }
  }
  // step04-障碍物数据轮廓提取
  auto cv_map = mathTools::EigenMatrixToCvMat(map_data_);
  //定义结构元素(这里使用3x3的矩形结构元素)
  cv::Mat dilate_element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  //进行闭运算
  cv::Mat dilated_map;
  cv::morphologyEx(cv_map, dilated_map, cv::MORPH_CLOSE, dilate_element);
  // stepe5->删格地图转全局障碍物
  map_data_ = mathTools : : CvMatToEigenMatrix(dilated_map);
  vector<Position> map_obs;
  for (int x = 0; x < size_(0); ++x) {
    for (int y = 0; y < size_(l); ++y) {
      if (map_data_(x, y) == 0) {
        continue;
      }
      float trans_x = x * resolution_ - back_height_;
      float trans_y = y * resolution_ - length_(l) / 2;
      port::CommonPose obs_pose(trans_x, trans_y, 0.f);
      auto g_obs_pose = mathTools::Rob2Global(sync_pose, obs_pose);
      Position new_map_obs;
      new_map_obs << g_obs_pose.x, g_obs_pose.y;
      map_obs.push_back(new_map_obs);
    }
  }
  return map_obs;
}

}  // namespace gridmap
}  // namespace modules
