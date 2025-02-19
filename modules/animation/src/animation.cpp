#include "animation.h"

#include <filesystem>
#include <iostream>
#include <matplotlibcpp17/cm.h>
#include <matplotlibcpp17/mplot3d.h>
#include <matplotlibcpp17/patches.h>
#include <opencv2/opencv.hpp>

#include "data_center.h"
#include "robot_configuration.h"
#include "tools/math_tools.h"

using namespace modules::datacenter;
namespace mpl_patches = matplotlibcpp17::patches;
namespace robot = modules::vehicle;
namespace mathTools = utilities::mathTools;
namespace cm = matplotlibcpp17::cm;

namespace modules {
namespace animation {

const float CMD_X_RANGE = 50;
const float MAP_RESOLUTION = 0.043f;
const int ENV_DURATION = 30;
const int DURATION = 100;

#define FIGURE_INIT(obj, fig_kwargs, axes_kwargs)                       \
  do {                                                                  \
    obj##_plt_ = mpl::pyplot::import();                                 \
    mpl::figure::Figure figure = obj##_plt_.figure(Args(), fig_kwargs); \
    obj##_figure_ptr_ = make_shared<mpl::figure::Figure>(figure);       \
    mpl::axes::Axes axes_obj = obj##_plt_.axes(axes_kwargs);            \
    obj##_axes_ptr_ = make_shared<mpl::axes::Axes>(axes_obj);           \
                                                                        \
    obj##_plt_.show(Args(), Kwargs("block"_a = 0));                     \
  } while (0)

DataCenter* DC_Instance = DataCenter::GetInstance();

// canvas and flush events
auto canvas_update_flush_events = [](pybind11::object figure) {
  pybind11::object canvas_attr = figure.attr("canvas");
  pybind11::object canvas_update_attr = canvas_attr.attr("update");
  pybind11::object canvas_flush_events_attr = canvas_attr.attr("flush_events");
  pybind11::object ret01 = canvas_update_attr();
  pybind11::object ret02 = canvas_flush_events_attr();
};

// canvas_copy_from_bbox
auto canvas_copy_from_bbox = [](pybind11::object figure) -> pybind11::object {
  pybind11::object canvas_attr = figure.attr("canvas");
  pybind11::object canvas_copy_from_bbox_attr = canvas_attr.attr("copy_from_bbox");
  pybind11::object fig_bbox = figure.attr("bbox");
  pybind11::object ret = canvas_copy_from_bbox_attr(fig_bbox);
  return ret;
};

// canvas_restore_region
auto canvas_restore_region = [](pybind11::object figure, pybind11::object bg) {
  pybind11::object canvas_attr = figure.attr("canvas");
  pybind11::object canvas_restore_region_attr = canvas_attr.attr("restore_region");
  canvas_restore_region_attr(bg);
};

void Animation::EnvPltInit(int ratio, const pybind11::dict& fig_kwargs, const port::CommonPose& robot_pose) {
  FIGURE_INIT(env, fig_kwargs, Kwargs());
  env_axes_ptr_->unwrap().attr("set_axis_off")();
  int pose_bias = 2;
  env_axes_ptr_->set_xlim(Args(robot_pose.x - ratio * pose_bias, robot_pose.x + ratio * pose_bias));
  env_axes_ptr_->set_ylim(Args(robot_pose.y - pose_bias, robot_pose.y + pose_bias));
  env_plt_.pause(Args(0.1));
  env_base_background_ = canvas_copy_from_bbox(env_figure_ptr_->unwrap());
  env_background_ = env_base_background_;
  // 颜色映射
  auto mpl_cm = pybind11::module::import("matplotlib.cm");
  jet_cmap_ = mpl_cm.attr("get_cmap")("jet");
}

void Animation::CmdPltInit(const pybind11::dict& fig_kwargs, const float& x_axis_range) {
  FIGURE_INIT(cmd, fig_kwargs, Kwargs("facecolor"_a = "lightsalmon"));
  cmd_plt_.grid(Args(true), Kwargs("linestyle"_a = "--", "linewidth"_a = 0.5, "color"_a = "black", "alpha"_a = 0.5));
  // cmd_axes_ptr_->unwrap().attr("set_axis_off")();
  cmd_axes_ptr_->set_xlim(Args(-0.3f, x_axis_range));
  cmd_axes_ptr_->set_ylim(Args(-1.5f, 2.5f));
  cmd_plt_.pause(Args(0.1));
  cmd_background_ = canvas_copy_from_bbox(cmd_figure_ptr_->unwrap());
}

void Animation::MapPltInit(const pybind11::dict& fig_kwargs, int ratio, const port::CommonPose& robot_pose) {
  // 检查文件是否存在
  string map_file_path = string{PROJECT_DIR} + "/maps/static_map.png";
  filesystem::path filePath = map_file_path;
  cv::Mat image1f;
  if (filesystem::exists(filePath)) {
    image1f = cv::imread(map_file_path, cv::IMREAD_UNCHANGED);
    // png图片转存(数据回放)
    const string csv_dir = string{CSV_SOURCE_DIR} + "/csvLog" + "/static_map.png";
    bool result = cv::imwrite(csv_dir, image1f);
    if (!result) {
      std::cerr << "Failed to save image to: " << csv_dir << std::endl;
    }
  } else {
    cout << "map file does not exist." << endl;
    map_init_flag_ = false;
  }

  // Create a NumPy array shape using the size of the OpenCV matrix
  std::vector<int> shape = {image1f.rows, image1f.cols, image1f.channels()};
  std::vector<size_t> strides = {image1f.step[0], image1f.step[1], image1f.step[2]};
  // Create a pybind11 array using the matrix data
  py::array_t<uint8_t> np_image = py::array(py::buffer_info(image1f.data,                              // Pointer to the data
                                                            sizeof(uint8_t),                           // size of one scalar
                                                            py::format_descriptor<uint8_t>::format(),  // Data type
                                                            3,                                         // Number of dimenstions
                                                            shape,                                     // shape
                                                            strides                                    // strides
                                                            ));

  // figure初始化
  map_plt_ = mpl::pyplot::import();
  mpl::figure::Figure figure_obj = map_plt_.figure(Args(), fig_kwargs);
  map_figure_ptr_ = make_shared<mpl::figure::Figure>(figure_obj);
  mpl::axes::Axes axes_obj = map_plt_.axes();
  map_axes_ptr_ = make_shared<mpl::axes::Axes>(axes_obj);
  // 背景设置
  map_plt_.show(Args(), Kwargs("block"_a = 0));
  map_axes_ptr_->unwrap().attr("set_axis_off")();
  map_axes_ptr_->imshow(Args(np_image), Kwargs("interpolation"_a = "bilinear", "origin"_a = "lower")).unwrap();
  int local_bias = 10;
  map_axes_ptr_->set_xlim(Args(robot_pose.x / MAP_RESOLUTION - local_bias * ratio, robot_pose.x / MAP_RESOLUTION + local_bias * ratio));
  map_axes_ptr_->set_ylim(Args(robot_pose.y / MAP_RESOLUTION - local_bias, robot_pose.y / MAP_RESOLUTION + local_bias));
  map_plt_.pause(Args(0.1));
  map_background_ = canvas_copy_from_bbox(map_figure_ptr_->unwrap());
  map_init_flag_ = true;
}

void Animation::SensorPltInit(const pybind11::dict& fig_kwargs, const float& offset) {
  FIGURE_INIT(sensor, fig_kwargs, Kwargs());
  sensor_axes_ptr_->unwrap().attr("set_axis_off")();
  sensor_axes_ptr_->set_xlim(Args(-1 * offset, offset));
  sensor_axes_ptr_->set_ylim(Args(-1 * offset, offset));
  sensor_plt_.pause(Args(0.1));
  sensor_background_ = canvas_copy_from_bbox(sensor_figure_ptr_->unwrap());
}

void Animation::SpacePltInit(const pybind11::dict& fig_kwargs) {
  space_plt_ = matplotlibcpp17::pyplot::import();
  matplotlibcpp17::mplot3d::import();
  auto figure = space_plt_.figure(Args(), fig_kwargs);
  space_figure_ptr_ = make_shared<mpl::figure::Figure>(figure);
  auto axes_obj = space_figure_ptr_->add_subplot(Args(), Kwargs("projection"_a = "3d"));
  space_axes_ptr_ = make_shared<mpl::axes::Axes>(axes_obj);
  space_plt_.show(Args(), Kwargs("block"_a = 0));
  space_axes_ptr_->set_xlim(Args(-1.95f, 1.95f));
  space_axes_ptr_->set_ylim(Args(-1.41f, 1.41f));
  space_axes_ptr_->set_zlim(Args(-0.75f, 0.75f));
  space_axes_ptr_->unwrap().attr("view_init")(30, -80);
  space_axes_ptr_->unwrap().attr("set_box_aspect")(py::make_tuple(1.3, 0.94, 0.5));
  space_plt_.pause(Args(0.1));
  space_background_ = canvas_copy_from_bbox(space_figure_ptr_->unwrap());
}

void Animation::InitializePlt(const port::CommonPose& robot_pose, const Mode& mode) {
  //模式设置
  running_mode_ = mode;
  //图框设置
  int fig_width, ratio;
  /***环境视图***/
  fig_width = 10, ratio = 1;
  pybind11::dict env_kwargs("figsize"_a = py::make_tuple(ratio * fig_width, fig_width), "dpi"_a = 100, "tight_layout"_a = true,
                            "facecolor"_a = "lightgray");
  EnvPltInit(ratio, env_kwargs, robot_pose);
  /***速度监视器***/
  pybind11::dict cmd_kwargs("figsize"_a = py::make_tuple(3, 3), "dpi"_a = 100, "tight_layout"_a = true);
  CmdPltInit(cmd_kwargs, CMD_X_RANGE);
  /***地图模块***/
  fig_width = 4, ratio = 1;
  pybind11::dict map_kwargs("figsize"_a = py::make_tuple(ratio * fig_width, fig_width), "dpi"_a = 100, "tight_layout"_a = true);
  MapPltInit(map_kwargs, ratio, robot_pose);
  /***传感器监视器***/
  pybind11::dict sensor_kwargs("figsize"_a = py::make_tuple(8, 8), "dpi"_a = 100, "tight_layout"_a = true, "facecolor"_a = "lavender");
  float offset = 11.f;
  SensorPltInit(sensor_kwargs, offset);
  /***三维监视器***/
  fig_width = 3, ratio = 1;
  pybind11::dict space_kwargs("figsize"_a = py::make_tuple(ratio * fig_width, fig_width), "dpi"_a = 100, "tight_layout"_a = true,
                              "facecolor"_a = "lightgray");
  SpacePltInit(space_kwargs);

  //机器人参数生成(三维参数需要计算)
  robot::InitRobotConfig();
}

void Animation::GetCommonData() {
  robot_pose_ = DC_Instance->GetRobotPose();
  cmd_ = DC_Instance->GetCmdVel();
  tracking_path_ = DC_Instance->GetPltTrackingPath();
  trajectory_ = DC_Instance->GetPltTrajectroy(200);
  sharp_curve_ = DC_Instance->GetPltSharpCurves();
  dg_border_ = DC_Instance->GetPltDangerousBorder();
  map_boundary_ = DC_Instance->GetPltBoundary();
  target_ = DC_Instance->GetTargetPose();
  orin_obs_ = DC_Instance->GetPlotObsPoints();
  sensor_obs_ = DC_Instance->GetAiObs();
  map_obs_ = DC_Instance->GetMapObs();
  sync_pose_ = DC_Instance->GetAiSyncRobotPose();
  erase_flag_ = false;
  blade_ = true;
}

void Animation::SetTrackingData(const port::CommonPose& robot_pose, const port::Twist& cmd, const mesh2D& path, const mesh2D& sp_curve,
                                const mesh2D& dg_border, const port::CommonPose& target, const bool erase, const bool blade) {
  robot_pose_ = robot_pose;
  DC_Instance->SetRobotPose(robot_pose);
  cmd_ = cmd;
  tracking_path_ = path;
  sharp_curve_ = sp_curve;
  dg_border_ = dg_border;
  target_ = target;
  erase_flag_ = erase;
  trajectory_ = DC_Instance->GetPltTrajectroy(200, erase_flag_);
  blade_ = blade;
}

vector<Eigen::Vector2f> Animation::ObsDataTrans(const vector<float>& obs_data) {
  vector<float> copy_data = obs_data;
  if (copy_data.size() % 2 != 0) {
    copy_data.pop_back();
  }
  int points_num = copy_data.size() / 2;
  vector<Eigen::Vector2f> trans_obs(points_num);
  for (uint i = 0; i < points_num; i++) {
    trans_obs[i](0) = copy_data[i];
    trans_obs[i](1) = copy_data[i + points_num];
  }
  return trans_obs;
}

void Animation::SetObsData(const port::CommonPose& sync_pose, const vector<float>& sensor_obs, const vector<float>& map_obs) {
  sync_pose_ = sync_pose;
  //传感器障碍物提取
  sensor_obs_ = ObsDataTrans(sensor_obs);
  //地图障碍物数据
  map_obs_ = ObsDataTrans(map_obs);
}

/*
 *@brief:绘制路径上的特征点（目标点，路径终点）
 *@param:
 *    axes:当前figure对应的axes
 */
void Animation::PlotPointsOnPath() {
  /*step01->实时数据更新*/
  mesh2D points(2);
  static py::object points_artist;
  // 跟踪目标点
  points[0].push_back(target_.x);
  points[1].push_back(target_.y);
  // 路径终点
  if (!tracking_path_.empty()) {
    points[0].push_back(tracking_path_[0].back());
    points[1].push_back(tracking_path_[1].back());
  }
  /*step02->static artist生成*/
  static bool once_flag = true;
  if (once_flag) {
    once_flag = false;
    pybind11::dict kwargs("c"_a = "r", "ls"_a = "None", "animated"_a = true, "marker"_a = "*", "markersize"_a = 6);
    points_artist = env_axes_ptr_->plot(Args(points[0], points[1]), kwargs).unwrap().cast<py::list>()[0];
  }
  /*step03->artist实时数据更新并绘制*/
  points_artist.attr("set_data")(points[0], points[1]);
  env_axes_ptr_->unwrap().attr("draw_artist")(points_artist);
}

/*
 *@brief:生成polygon artist
 *@params:
 *    points:polygon边界点
 *    color:颜色设置
 *    alpha:透明度
 */
py::object Animation::DefinePoly(const vector<vector<float>>& points, string color, float alpha) {
  auto poly = py::array(py::cast(std::move(points)));
  auto polygon = mpl_patches::Polygon(Args(poly, true), Kwargs("color"_a = color, "alpha"_a = alpha));
  return polygon.unwrap();
}

/*
 *@brief:曲线绘制（路径，轨迹，急转弯）
 *@params:
 *	  axes:图像axes
 *	  robot_pose:实时定位
 */
void Animation::Plotline() {
  /*step01->实时数据更新*/
  vector<mesh2D> lines(4);
  static vector<py::object> lines_artist(4);
  lines[0] = tracking_path_;
  lines[1] = trajectory_;
  lines[2] = sharp_curve_;
  lines[3] = dg_border_;
  // lines[4] = map_boundary_;
  static py::object vector_artist;
  /*step02->static artist生成*/
  static bool once_flag = true;
  if (once_flag) {
    once_flag = false;
    vector<string> colors = {"k", "g", "deeppink", "lightcyan", "indigo"};
    vector<string> marker = {"", "", "o", "o", "o"};
    vector<int> size = {0, 0, 1, 5, 2};
    vector<float> alpha = {1, 1, 1, 0.1, 1};
    for (int i = 0; i < lines.size(); i++) {
      if (i >= 2) {
        pybind11::dict kwargs("c"_a = colors[i], "ls"_a = "None", "animated"_a = true, "marker"_a = marker[i], "alpha"_a = alpha[i],
                              "markersize"_a = size[i]);
        lines_artist[i] = env_axes_ptr_->plot(Args(lines[i][0], lines[i][1]), kwargs).unwrap().cast<py::list>()[0];
        continue;
      }
      lines_artist[i] = env_axes_ptr_->plot(Args(lines[i][0], lines[i][1]), Kwargs("c"_a = colors[i], "lw"_a = 1.0)).unwrap().cast<py::list>()[0];
    }
    // 显示路径方向信息
    pybind11::dict Kwargs("color"_a = "coral", "scale"_a = 30, "units"_a = "width", "animated"_a = true, "width"_a = 0.002f, "headwidth"_a = 2,
                          "headlength"_a = 4, "alpha"_a = 1);
    vector_artist = env_axes_ptr_->quiver(Args(0.f, 0.f, 1.f, 1.f), Kwargs).unwrap();
  }
  /*step03->artist实时数据更新并绘制*/
  for (int j = 0; j < lines_artist.size(); j++) {
    lines_artist[j].attr("set_data")(lines[j][0], lines[j][1]);
    env_axes_ptr_->unwrap().attr("draw_artist")(lines_artist[j]);
  }
  //路径方向(起点)
  if (!lines[0].empty() && lines[0][0].size() > 2) {
    vector_artist.attr("set_offsets")(py::make_tuple(lines[0][0][0], lines[0][1][0]));
    Eigen::Vector2f path_dir(lines[0][0][1] - lines[0][0][0], lines[0][1][1] - lines[0][1][0]);
    Eigen::Vector2f path_dir_norm = path_dir.normalized();
    vector<float> u, v;
    u = {path_dir_norm[0]};
    v = {path_dir_norm[1]};
    vector_artist.attr("set_UVC")(u, v);
    env_axes_ptr_->unwrap().attr("draw_artist")(vector_artist);
  }
}

void Animation::PlotRobotOutline(const Scene& scene) {
  port::CommonPose modify_pose;
  if (scene == Scene::ENV) {
    modify_pose = robot_pose_;
  } else {
    modify_pose = port::CommonPose(0.f, 0.f, sync_pose_.theta);
  }
  /*step01->实时数据更新*/
  int num = 5;
  vector<mesh2D> outlines(num);  // 机身轮廓：外轮廓+左驱动轮+右驱动轮+左从动轮+右从动轮
  static vector<py::object> env_outlines(num);
  static py::object robot_boundary;
  static vector<py::object> sensor_outlines(num);
  // static py::object sensor_robot_boundary;
  outlines[0] = DC_Instance->GetRobotOutline(modify_pose);
  outlines[1] = DC_Instance->GetRobotDriveWheel(modify_pose, 1);
  outlines[2] = DC_Instance->GetRobotDriveWheel(modify_pose, -1);
  outlines[3] = DC_Instance->GetRobotOmniWheel(modify_pose, 1, cmd_.angular);
  outlines[4] = DC_Instance->GetRobotOmniWheel(modify_pose, -1, cmd_.angular);
  /*step02->static artist生成*/
  static bool once_flag = true;
  if (once_flag) {
    once_flag = false;
    vector<string> color = {"b", "k", "k", "k", "k"};
    vector<float> alpha = {0.2f, 0.7f, 0.7f, 0.7f, 0.7f};
    for (int i = 0; i < num; i++) {
      env_outlines[i] = DefinePoly(outlines[i], color[i], alpha[i]);
      sensor_outlines[i] = DefinePoly(outlines[i], color[i], alpha[i]);
      env_axes_ptr_->add_patch(Args(env_outlines[i]));
      sensor_axes_ptr_->add_patch(Args(sensor_outlines[i]));
    }
    // 机器轮廓加深
    pybind11::dict kwargs("c"_a = "r", "ls"_a = "-", "lw"_a = 2, "animated"_a = true);
    robot_boundary = env_axes_ptr_->plot(Args(outlines[1][0], outlines[1][1]), kwargs).unwrap().cast<py::list>()[0];
    // sensor_robot_boundary = sensor_axes_ptr_->plot(Args(outlines[1][0], outlines[1][1]), kwargs).unwrap().cast<py::list>()[0];
  }
  /*step03->artist实时数据更新并绘制*/
  for (int j = 0; j < num; j++) {
    auto outline_array = py::array(py::cast(std::move(outlines[j])));
    auto outline = scene == Scene::ENV ? env_outlines[j] : sensor_outlines[j];
    outline.attr("set_xy")(outline_array);
    auto axes_ptr = scene == Scene::ENV ? env_axes_ptr_ : sensor_axes_ptr_;
    axes_ptr->unwrap().attr("draw_artist")(outline);
  }
  // 外轮廓加深
  vector<float> x_array, y_array;
  for (auto point : outlines[0]) {
    x_array.push_back(point[0]);
    y_array.push_back(point[1]);
  }
  // if (scene == Scene::ENV) {
  //   robot_boundary.attr("set_data")(x_array, y_array));
  //   env_axes_ptr_->unwrap().attr("draw_artist")(robot_boundary);
  // }
}

void Animation::PlotTargetOutline() {
  /*step01->实时数据更新*/
  static py::object robot_boundary;
  auto pose = DC_Instance->GetTestTempPose();
  mesh2D outlines = DC_Instance->GetRobotOutline(pose);
  /*step02->static artist生成*/
  static bool once_flag = true;
  if (once_flag) {
    once_flag = false;
    // 机器轮廓加深
    pybind11::dict kwargs("c"_a = "r", "ls"_a = "-", "lw"_a = 2, "animated"_a = true);
    robot_boundary = env_axes_ptr_->plot(Args(outlines[0], outlines[1]), kwargs).unwrap().cast<py::list>()[0];
  }
  /*step03->artist实时数据更新并绘制*/
  vector<float> x_array, y_array;
  for (auto point : outlines) {
    x_array.push_back(point[0]);
    y_array.push_back(point[1]);
  }
  robot_boundary.attr("set_data")(x_array, y_array);
  env_axes_ptr_->unwrap().attr("draw_artist")(robot_boundary);
}

void Animation::PlotVisionBoundary() {
  /*step01->实时数据更新*/
  auto local_pose = port::CommonPose(0.0f, 0.0f, sync_pose_.theta);
  vector<mesh2D> vision_boundary = DC_Instance->GetVisionBoundary(local_pose);
  static vector<py::object> vision_polygon;
  vision_polygon.resize(vision_boundary.size());
  /*step02->static artist生成*/
  static bool once_flag = true;
  if (once_flag) {
    once_flag = false;
    py::dict bound_kwargs("c"_a = "k", "ls"_a = "-.");
    // 视觉窗口
    if (!vision_boundary.empty()) {
      for (int i = 0; i < vision_boundary.size(); i++) {
        py::tuple bound_args = py::make_tuple(vision_boundary[i][0], vision_boundary[i][1]);
        vision_polygon[i] = sensor_axes_ptr_->plot(bound_args, bound_kwargs).unwrap().cast<py::list>()[0];
      }
    }
  }
  /*step03->artist实时数据更新并绘制*/
  // 视觉窗口
  if (!vision_boundary.empty()) {
    for (int j = 0; j < vision_polygon.size(); j++) {
      vision_polygon[j].attr("set_data")(vision_boundary[j][0], vision_boundary[j][1]);
      sensor_axes_ptr_->unwrap().attr("draw_artist")(vision_polygon[j]);
    }
  }
}

/*
 *@brief:显示激光雷达数据
 */
void Animation::PlotLidarData() {
  //雷达射线显示
  mesh2D lidar_data = DC_Instance->GetLidarData();
  /*step02->static artist生成*/
  static vector<py::object> lidar_ray_artist(lidar_data[0].size());
  static bool once_flag = true;
  if (once_flag) {
    once_flag = false;
    pybind11::dict kwargs("c"_a = "gray", "ls"_a = "-", "lw"_a = 0.5, "animated"_a = true, "alpha"_a = 0.5);
    for (uint i = 0; i < lidar_data[0].size(); i++) {
      vector<float> ray_x(2), ray_y(2);
      ray_x[0] = 0.0;
      ray_y[0] = 0.0;
      ray_x[1] = lidar_data[0][i] - sync_pose_.x;
      ray_y[1] = lidar_data[1][i] - sync_pose_.y;
      lidar_ray_artist[i] = sensor_axes_ptr_->plot(Args(ray_x, ray_y), kwargs).unwrap().cast<py::list>()[0];
    }
  }
  /*step03->artist实时数据更新并绘制*/
  for (uint i = 0; i < lidar_data[0].size(); i++) {
    vector<float> ray_x(2), ray_y(2);
    ray_x[0] = 0.0;
    ray_y[0] = 0.0;
    ray_x[1] = lidar_data[0][i] - sync_pose_.x;
    ray_y[1] = lidar_data[1][i] - sync_pose_.y;
    lidar_ray_artist[i].attr("set_data")(ray_x, ray_y);
    sensor_axes_ptr_->unwrap().attr("draw_artist")(lidar_ray_artist[i]);
  }
}

void Animation::PlotGridMapObs() {
  /*step01->实时数据更新*/
  mesh2D map_points(2);
  for (auto point : map_obs_) {
    float local_x = point(0) - sync_pose_.x;
    float local_y = point(1) - sync_pose_.y;
    map_points[0].push_back(local_x);
    map_points[1].push_back(local_y);
  }
  /*step02->static artist生成*/
  static py::object map_obs_artist;
  static bool once_flag = true;
  if (once_flag) {
    once_flag = false;
    pybind11::dict kwargs2("c"_a = "g", "ls"_a = "None", "animated"_a = true, "marker"_a = "o", "markersize"_a = 3, "alpha"_a = 0.3);
    map_obs_artist = sensor_axes_ptr_->plot(Args(map_points[0], map_points[1]), kwargs2).unwrap().cast<py::list>()[0];
  }
  /*step03->artist实时数据更新并绘制*/
  map_obs_artist.attr("set_data")(map_points[0], map_points[1]);
  sensor_axes_ptr_->unwrap().attr("draw_artist")(map_obs_artist);
}

void Animation::PlotEntityObs(const Scene& scene) {
  port::CommonPose transform_pose;
  if (scene == Scene::SENSOR) {
    transform_pose = port::CommonPose(sync_pose_.x, sync_pose_.y, sync_pose_.theta);
  } else {
    transform_pose = port::CommonPose(0.f, 0.f, sync_pose_.theta);
  }
  /*step01->实时数据更新*/
  auto show_obs = orin_obs_;
  int obs_num = show_obs.size();
  vector<mesh2D> orin_obs(obs_num);
  for (uint j = 0; j < obs_num; j++) {
    orin_obs[j].resize(2);
    vector<float> one_obs_x, one_obs_y;
    one_obs_x = show_obs[j].first;
    one_obs_y = show_obs[j].second;
    for (int i = 0; i < one_obs_x.size(); i++) {
      float local_x = one_obs_x[i] - transform_pose.x;
      float local_y = one_obs_y[i] - transform_pose.y;
      orin_obs[j][0].push_back(local_x);
      orin_obs[j][1].push_back(local_y);
    }
  }
  /*step02->static artist生成*/
  static vector<py::object> env_obs_artist(obs_num);
  static vector<py::object> sensor_obs_artist(obs_num);
  static bool once_flag = true;
  if (once_flag) {
    once_flag = false;
    pybind11::dict kwargs("c"_a = "gray", "ls"_a = "-", "animated"_a = true, "alpha"_a = 0.5);
    for (uint i = 0; i < obs_num; i++) {
      env_obs_artist[i] = env_axes_ptr_->plot(Args(orin_obs[i][0], orin_obs[i][1]), kwargs).unwrap().cast<py::list>()[0];
      sensor_obs_artist[i] = sensor_axes_ptr_->plot(Args(orin_obs[i][0], orin_obs[i][1]), kwargs).unwrap().cast<py::list>()[0];
    }
  }
  /*step03->artist实时数据更新并绘制*/
  auto obs_artist = scene == Scene::ENV ? env_obs_artist : sensor_obs_artist;
  auto local_axes_ptr_ = scene == Scene::ENV ? env_axes_ptr_ : sensor_axes_ptr_;
  for (uint i = 0; i < obs_num; i++) {
    obs_artist[i].attr("set_data")(orin_obs[i][0], orin_obs[i][1]);
    local_axes_ptr_->unwrap().attr("draw_artist")(obs_artist[i]);
  }
}

/*
 *@brief:传感器识别障碍物绘制
 */
void Animation::PlotSensorObs(const Scene& scene) {
  port::CommonPose transform_pose;
  if (scene == Scene::ENV) {
    transform_pose = port::CommonPose(sync_pose_.x, sync_pose_.y, sync_pose_.theta);
  } else {
    transform_pose = port::CommonPose(0.f, 0.f, sync_pose_.theta);
  }
  /*step01->实时数据更新*/
  vector<float> obs_x, obs_y;
  //障碍物数据坐标变换(传感器坐标系->全局坐标系）
  for (auto point : sensor_obs_) {
    port::CommonPose obs_pose(point(0), point(1), 0.f);
    auto transed_pose = mathTools::Rob2Global(transform_pose, obs_pose);
    obs_x.push_back(transed_pose.x);
    obs_y.push_back(transed_pose.y);
  }
  /*step02->static artist生成*/
  static py::object env_obs_artist, sensor_obs_artist;
  static bool once_flag = true;
  if (once_flag) {
    once_flag = false;
    pybind11::dict kwargs("c"_a = "r", "ls"_a = "None", "animated"_a = true, "marker"_a = "o", "markersize"_a = 2, "alpha"_a = 0.5);
    env_obs_artist = env_axes_ptr_->plot(Args(obs_x, obs_y), kwargs).unwrap().cast<py::list>()[0];
    sensor_obs_artist = sensor_axes_ptr_->plot(Args(obs_x, obs_y), kwargs).unwrap().cast<py::list>()[0];
  }
  /*step03->artist实时数据更新并绘制*/
  py::object obs_artist = scene == Scene::ENV ? env_obs_artist : sensor_obs_artist;
  auto local_axes_ptr_ = scene == Scene::ENV ? env_axes_ptr_ : sensor_axes_ptr_;
  obs_artist.attr("set_data")(obs_x, obs_y);
  local_axes_ptr_->unwrap().attr("draw_artist")(obs_artist);
}

/**
 *@brief:设置轴边界
 *@params:
 *   axes:图像axes
 *	 robot_pose:实时定位
 *   traj:历史轨迹
 *	 offset:轴偏差
 */
void Animation::SetEnvAxisLimit(const mesh2D& traj, const int ratio, float offset) {
  // //以机器为中心
  // offset = 10.f;
  // env_axes_ptr_->set_xlim(Args(robot_pose_.x - offset / 2, robot_pose_.x + offset / 2));
  // env_axes_ptr_->set_ylim(Args(robot_pose_.y - offset / (ratio * 2), robot_pose_.y + offset / (ratio * 2)));
  // return;
  /*********step01->计算x/y轴数据边界**********/
  // 原始axis边界
  auto axes_xlim = env_axes_ptr_->get_xlim();
  pybind11::list axes_ylim = env_axes_ptr_->unwrap().attr("get_ylim")();
  float axis_min_x, axis_max_x, axis_min_y, axis_max_y;
  axis_min_x = get<0>(axes_xlim);
  axis_max_x = get<1>(axes_xlim);
  axis_min_y = axes_ylim[0].cast<float>();
  axis_max_y = axes_ylim[1].cast<float>();
  //计算外轮廓x轴方向极值
  float outline_min_x, outline_max_x, outline_min_y, outline_max_y;
  outline_min_x = outline_max_x = robot_pose_.x;
  outline_min_y = outline_max_y = robot_pose_.y;
  mesh2D outlines = DC_Instance->GetRobotOutline(robot_pose_);
  for (auto point : outlines) {
    if (point[0] < outline_min_x) {
      outline_min_x = point[0];
    }
    if (point[0] > outline_max_x) {
      outline_max_x = point[0];
    }
    if (point[1] < outline_min_y) {
      outline_min_y = point[1];
    }
    if (point[1] > outline_max_y) {
      outline_max_y = point[1];
    }
  }
  //计算路径x轴方向极值
  float path_min_x, path_max_x, path_min_y, path_max_y;
  path_min_x = path_max_x = robot_pose_.x;
  path_min_y = path_max_y = robot_pose_.y;
  mesh2D path = tracking_path_;
  if (path[0].size() > 5) {
    auto p_max_x = std::max_element(path[0].begin(), path[0].end());
    auto p_min_x = std::min_element(path[0].begin(), path[0].end());
    auto p_max_y = std::max_element(path[1].begin(), path[1].end());
    auto p_min_y = std::min_element(path[1].begin(), path[1].end());
    path_min_x = min(path_min_x, *p_min_x);
    path_max_x = max(path_max_x, *p_max_x);
    path_min_y = min(path_min_y, *p_min_y);
    path_max_y = max(path_max_y, *p_max_y);
  }
  //计算障碍物x轴方向极值
  float obs_min_x, obs_max_x, obs_min_y, obs_max_y;
  obs_min_x = obs_max_x = robot_pose_.x;
  obs_min_y = obs_max_y = robot_pose_.y;
  auto show_obs = orin_obs_;
  for (int i = 0; i < show_obs.size(); i++) {
    auto max_x = std::max_element(show_obs[i].first.begin(), show_obs[i].first.end());
    auto min_x = std::min_element(show_obs[i].first.begin(), show_obs[i].first.end());
    auto max_y = std::max_element(show_obs[i].second.begin(), show_obs[i].second.end());
    auto min_y = std::min_element(show_obs[i].second.begin(), show_obs[i].second.end());
    obs_min_x = min(*min_x, obs_min_x);
    obs_max_x = max(*max_x, obs_max_x);
    obs_min_y = min(*min_y, obs_min_y);
    obs_max_y = max(*max_y, obs_max_y);
  }
  //计算轨迹x方向极值
  auto traj_max_x = *(std::max_element(traj[0].begin(), traj[0].end()));
  auto traj_min_x = *(std::min_element(traj[0].begin(), traj[0].end()));
  auto traj_max_y = *(std::max_element(traj[1].begin(), traj[1].end()));
  auto traj_min_y = *(std::min_element(traj[1].begin(), traj[1].end()));

  /*********step02->x/y数据边界比较**********/
  float x_min_lim = min(axis_min_x, min(outline_min_x, min(obs_min_x, min(path_min_x, traj_min_x))));
  float x_max_lim = max(axis_max_x, max(outline_max_x, max(obs_max_x, max(path_max_x, traj_max_x))));
  float y_min_lim = min(axis_min_y, min(outline_min_y, min(obs_min_y, min(path_min_y, traj_min_y))));
  float y_max_lim = max(axis_max_y, max(outline_max_y, max(obs_max_y, max(path_max_y, traj_max_y))));

  axis_change_ = false;
  if (x_min_lim < axis_min_x) {
    axis_change_ = true;
    x_min_lim = x_min_lim - offset;
  }
  if (x_max_lim > axis_max_x) {
    axis_change_ = true;
    x_max_lim = x_max_lim + offset;
  }
  if (y_min_lim < axis_min_y) {
    axis_change_ = true;
    y_min_lim = y_min_lim - offset / ratio;
  }
  if (y_max_lim > axis_max_y) {
    axis_change_ = true;
    y_max_lim = y_max_lim + offset / ratio;
  }
  float x_axis_range = x_max_lim - x_min_lim;
  float y_axis_range = ratio * (y_max_lim - y_min_lim);
  if (axis_change_) {
    if (x_axis_range > y_axis_range) {
      env_axes_ptr_->set_xlim(Args(x_min_lim, x_max_lim));
      env_axes_ptr_->set_ylim(Args(y_min_lim, y_min_lim + x_axis_range / ratio));
    } else {
      env_axes_ptr_->set_xlim(Args(x_min_lim, x_min_lim + y_axis_range));
      env_axes_ptr_->set_ylim(Args(y_min_lim, y_max_lim));
    }
  }
}

bool Animation::FrequencyCtrl(int T, int64_t& last_time_stamp) {
  int64_t current_time_stamp = TimeToolKit::TimeSpecSysCurrentMs();  // 获取当前时间戳
  // 时间控制
  int record_time = current_time_stamp - last_time_stamp;
  if (record_time < T && last_time_stamp != 0) {
    return true;
  }
  last_time_stamp = current_time_stamp;
  return false;
}

void Animation::PlotMPCHrizon() {
  /*step01->实时数据更新*/
  mesh2D points(2);
  static py::object points_artist;
  auto horizon_states = DC_Instance->GetMpcHorizon();
  for (auto state : horizon_states) {
    points[0].push_back(state.x);
    points[1].push_back(state.y);
  }
  /*step02->static artist生成*/
  static bool once_flag = true;
  if (once_flag) {
    once_flag = false;
    pybind11::dict kwargs("c"_a = "springgreen", "ls"_a = "-", "lw"_a = 5, "animated"_a = true);
    points_artist = env_axes_ptr_->plot(Args(points[0], points[1]), kwargs).unwrap().cast<py::list>()[0];
  }
  /*step03->artist实时数据更新并绘制*/
  points_artist.attr("set_data")(points[0], points[1]);
  env_axes_ptr_->unwrap().attr("draw_artist")(points_artist);
}

void Animation::DynSysAvoidDisplay() {
  auto avoid_agent = DC_Instance->GetDsAvoidAgents();
  PlotAvoidAgents(avoid_agent);
  // PlotAvoidAgentsConvex();
  PlotObsSandCPoints();
  PlotC1VirtualObs();
  for (auto agent : avoid_agent) {
    // PlotAgentObsQuiver(agent);
    PlotAgentVelVector(agent);
  }
}

/**
 *@brief:绘制机器避障agentavoid_agent_s(轮廓，安全边界)
 *@params:
 *    axes:图像axes
 */
void Animation::PlotAvoidAgents(const vector<port::CircleAgent>& avoid_agents) {
  if (avoid_agents[0].calculated_obstacles.size() == 0) return;
  /*step01->实时数据更新*/
  static vector<py::object> virtual_boundary_artist(avoid_agents.size());
  static vector<py::object> safe_boundary_artist(avoid_agents.size());
  vector<Eigen::Vector2f> centers(avoid_agents.size());
  /*step02->static artist生成*/
  static bool once_flag = true;
  if (once_flag) {
    once_flag = false;
    pybind11::dict kwargs1("animated"_a = true, "facecolor"_a = "r", "alpha"_a = 0.5);
    pybind11::dict kwargs2("animated"_a = true, "facecolor"_a = "lime", "alpha"_a = 0.3);
    for (int i = 0; i < avoid_agents.size(); i++) {
      centers[i] = avoid_agents[i].center_pose;
      // 虚拟边界
      float robot_margin = avoid_agents[i].circle.radius;
      virtual_boundary_artist[i] = mpl_patches::Circle(Args(py::make_tuple(centers[i][0], centers[i][1]), robot_margin), kwargs1).unwrap();
      env_axes_ptr_->add_patch(Args(virtual_boundary_artist[i]));
      // 安全膨胀边界
      float safe_margin = avoid_agents[i].circle.radius + avoid_agents[i].circle.safe_margin;
      safe_boundary_artist[i] = mpl_patches::Circle(Args(py::make_tuple(centers[i][0], centers[i][1]), safe_margin), kwargs2).unwrap();
      env_axes_ptr_->add_patch(Args(safe_boundary_artist[i]));
    }
  }
  /*step03->artist实时数据更新并绘制*/
  for (int i = 0; i < avoid_agents.size(); i++) {
    centers[i] = avoid_agents[i].center_pose;
    // 虚拟边界
    virtual_boundary_artist[i].attr("set_center")(py::make_tuple(centers[i][0], centers[i][1]));
    env_axes_ptr_->unwrap().attr("draw_artist")(virtual_boundary_artist[i]);
    // 安全膨胀边界
    safe_boundary_artist[i].attr("set_center")(py::make_tuple(centers[i][0], centers[i][1]));
    env_axes_ptr_->unwrap().attr("draw_artist")(safe_boundary_artist[i]);
  }
}

void Animation::PlotAvoidAgentsConvex() {
  /*step01->实时数据更新*/
  static vector<py::object> boundary_artist(4);
  vector<Eigen::Vector2f> centers(4);
  for (int i = 0; i < 4; i++) {
    Eigen::Vector2f global_vec;
    global_vec(0) = robot::outline_points[i][0] * cos(robot_pose_.theta) - robot::outline_points[i][1] * sin(robot_pose_.theta) + robot_pose_.x;
    global_vec(1) = robot::outline_points[i][0] * sin(robot_pose_.theta) + robot::outline_points[i][1] * cos(robot_pose_.theta) + robot_pose_.y;
    centers[i] = global_vec;
  }
  /*step02->static artist生成*/
  static bool once_flag = true;
  if (once_flag) {
    once_flag = false;
    pybind11::dict kwargs("animated"_a = true, "facecolor"_a = "r", "alpha"_a = 0.5);
    for (int i = 0; i < boundary_artist.size(); i++) {
      // 虚拟边界
      float robot_margin = 0.15f;
      boundary_artist[i] = mpl_patches::Circle(Args(py::make_tuple(centers[i][0], centers[i][1]), robot_margin), kwargs).unwrap();
      env_axes_ptr_->add_patch(Args(boundary_artist[i]));
    }
  }
  /*step03->artist实时数据更新并绘制*/
  for (int i = 0; i < boundary_artist.size(); i++) {
    // 虚拟边界
    boundary_artist[i].attr("set_center")(py::make_tuple(centers[i][0], centers[i][1]));
    env_axes_ptr_->unwrap().attr("draw_artist")(boundary_artist[i]);
  }
}

/**
 *@brief:障碍物采样点显示
 *@params:
 * axes:图像axes
 */
void Animation::PlotObsSandCPoints() {
  /*step01->实时数据更新*/
  // 采样点，聚类障碍物点，聚类几何中心，最近障碍点
  auto sample_points = DC_Instance->GetAvoidSamplePoints();
  auto cluster_groups = DC_Instance->GetAvoidClusterGroup();
  auto obs_centers = DC_Instance->GetAvoidVirtualObsCenter();
  auto obs_nearest_points = DC_Instance->GetAvoidObsNearestPoints();
  vector<mesh2D> dispose_points(4);
  static vector<py::object> obs_points_artist(4);
  for (int i = 0; i < 4; i++) {
    dispose_points[i].resize(2);
  }
  // 采样点
  for (auto point : sample_points) {
    if (fabs(point.pose(0)) > 0.1f || fabs(point.pose(1)) > 0.1f) {
      dispose_points[0][0].push_back(point.pose(0));
      dispose_points[0][1].push_back(point.pose(1));
    }
  }
  // 聚类点
  for (auto group : cluster_groups) {
    for (auto point : group) {
      if (fabs(point.pose(0)) > 0.1f || fabs(point.pose(1)) > 0.1f) {
        dispose_points[1][0].push_back(point.pose(0));
        dispose_points[1][1].push_back(point.pose(1));
      }
    }
  }
  // 障碍物中心
  for (auto point : obs_centers) {
    if (fabs(point(0)) > 0.1f || fabs(point(1)) > 0.1f) {
      dispose_points[2][0].push_back(point(0));
      dispose_points[2][1].push_back(point(1));
    }
  }
  // 障碍物最近点
  for (auto point : obs_nearest_points) {
    if (fabs(point(0)) > 0.1f || fabs(point(1)) > 0.1f) {
      dispose_points[3][0].push_back(point(0));
      dispose_points[3][1].push_back(point(1));
    }
  }
  /*step02->static artist生成*/
  static bool once_flag = true;
  static vector<string> points_colors = {"dimgray", "darkorange", "blue", "orangered"};
  static vector<string> markers = {"o", "o", "X", "H"};
  static vector<int> marker_size = {2, 4, 6, 8};
  static vector<float> alpha = {1.0, 0.3, 1.0, 1.0};
  if (once_flag) {
    once_flag = false;
    for (int i = 0; i < dispose_points.size(); i++) {
      pybind11::dict kwargs("c"_a = points_colors[i], "ls"_a = "None", "animated"_a = true, "marker"_a = markers[i], "markersize"_a = marker_size[i],
                            "alpha"_a = alpha[i]);
      obs_points_artist[i] = env_axes_ptr_->plot(Args(dispose_points[i][0], dispose_points[i][1]), kwargs).unwrap().cast<py::list>()[0];
    }
  }
  /*step03->artist实时数据更新并绘制*/
  for (int j = 0; j < obs_points_artist.size(); j++) {
    // if(j > 1) continue;
    obs_points_artist[j].attr("set_data")(dispose_points[j][0], dispose_points[j][1]);
    env_axes_ptr_->unwrap().attr("draw_artist")(obs_points_artist[j]);
  }
}

void Animation::PlotC1VirtualObs() {
  /*step01->实时数据更新(虚拟C1连续障碍物)*/
  int temp_obs_num = 10;
  vector<mesh2D> obs_points(10);
  static vector<py::object> obs_points_artist(temp_obs_num);
  for (int i = 0; i < temp_obs_num; i++) {
    obs_points[i].resize(2);
  }
  auto virtual_obstacles = DC_Instance->GetAvoidVirtualObs();
  // 障碍物特征点数据处理
  int real_obs_num = virtual_obstacles.size();
  for (int i = 0; i < real_obs_num; i++) {
    if (i >= temp_obs_num) {
      cout << "!!!real obs num out of range!!!" << endl;
      break;
    }
    // 虚拟障碍物点提取
    for (auto point : virtual_obstacles[i].points) {
      if (fabs(point(0)) > 0.1f || fabs(point(1)) > 0.1f) {
        obs_points[i][0].push_back(point(0));
        obs_points[i][1].push_back(point(1));
      }
    }
    // // 保证轮廓闭合
    // obs_points[i][0].push_back(obs_points[i][0].front());
    // obs_points[i][1].push_back(obs_points[i][1].front());
  }
  /*step02->static artist生成*/
  static bool once_flag = true;
  if (once_flag) {
    once_flag = false;
    for (int i = 0; i < obs_points.size(); i++) {
      pybind11::dict kwargs("c"_a = "r", "ls"_a = "-", "lw"_a = 2, "marker"_a = "o", "markersize"_a = 3, "animated"_a = true);
      obs_points_artist[i] = env_axes_ptr_->plot(Args(obs_points[i][0], obs_points[i][1]), kwargs).unwrap().cast<py::list>()[0];
    }
  }

  /*step03->artist实时数据更新并绘制*/
  for (int j = 0; j < real_obs_num; j++) {
    float ratio_dis = virtual_obstacles[j].min_ratio_dis;
    float color_ratio = 1.0f - ratio_dis;
    auto local_color = jet_cmap_(color_ratio);
    obs_points_artist[j].attr("set_color")(local_color);
    obs_points_artist[j].attr("set_data")(obs_points[j][0], obs_points[j][1]);
    env_axes_ptr_->unwrap().attr("draw_artist")(obs_points_artist[j]);
  }
}

/**
 *@brief:机器虚拟轮廓+虚拟控制中心速度矢量显示
 *@params:
 *    axes:图像axes
 */
void Animation::PlotAgentVelVector(const port::CircleAgent& agent) {
  if (agent.center_pose(0) == 0.0 && agent.center_pose(1) == 0.0) return;
  /*step01->实时数据更新*/
  Eigen::Vector2f control_point = agent.center_pose;
  // 控制速度矢量
  static vector<py::object> vel_vector_artist(4);
  vector<Eigen::Vector2f> plt_vel_vector(4);
  plt_vel_vector[0] = agent.ref_vel_vec;  //期望速度方向
  // plt_vel_vector[0] = agent.cur_vel_vec;
  plt_vel_vector[1] = agent.compound_vel;  //最终合成速度方向
  plt_vel_vector[2] = agent.debug01_vec;
  plt_vel_vector[3] = agent.debug02_vec;
  /*step02->static artist生成*/
  static bool once_flag = true;
  if (once_flag) {
    once_flag = false;
    // 速度矢量
    vector<string> quiver_colors = {"gray", "indigo", "y", "b"};
    vector<float> quiver_width = {0.004f, 0.002f, 0.003f, 0.002f};
    vector<int> scales = {5, 5, 5, 5};
    vector<float> alpha = {0.4, 0.8, 1.0, 0.5};
    for (int i = 0; i < vel_vector_artist.size(); i++) {
      pybind11::dict Kwargs("color"_a = quiver_colors[i], "scale"_a = scales[i], "units"_a = "width", "animated"_a = true,
                            "width"_a = quiver_width[i], "headwidth"_a = 3, "headlength"_a = 5, "alpha"_a = alpha[i]);
      vel_vector_artist[i] =
          env_axes_ptr_->quiver(Args(control_point[0], control_point[1], plt_vel_vector[i][0], plt_vel_vector[i][1]), Kwargs).unwrap();
    }
  }
  /*step03->artist实时数据更新并绘制*/
  // 速度矢量
  for (int j = 0; j < vel_vector_artist.size(); j++) {
    vel_vector_artist[j].attr("set_offsets")(py::make_tuple(control_point[0], control_point[1]));
    vector<float> u, v;
    u = {plt_vel_vector[j][0]};
    v = {plt_vel_vector[j][1]};
    vel_vector_artist[j].attr("set_UVC")(u, v);
    env_axes_ptr_->unwrap().attr("draw_artist")(vel_vector_artist[j]);
  }
}

/**
 *@brief:障碍物采样点对应vector显示
 *@params:
 *    axes:图像axes
 */
void Animation::PlotAgentObsQuiver(const port::CircleAgent& agent) {
  /*step01->实时数据更新*/
  static vector<py::object> obs_vector_artist(3);
  auto obstacles = agent.calculated_obstacles;
  vector<vector<vector<float>>> vector_item_array(obs_vector_artist.size());
  for (auto i = 0; i < 3; i++) {
    vector_item_array[i].resize(4);
    for (auto obstacle : obstacles) {
      vector_item_array[i][0].push_back(obstacle.reference_point(0));
      vector_item_array[i][1].push_back(obstacle.reference_point(1));
      if (i == 0) {
        vector_item_array[i][2].push_back(obstacle.reference_vector(0));
        vector_item_array[i][3].push_back(obstacle.reference_vector(1));
      } else if (i == 1) {
        vector_item_array[i][2].push_back(obstacle.tangent_vector(0));
        vector_item_array[i][3].push_back(obstacle.tangent_vector(1));
      } else if (i == 2) {
        vector_item_array[i][2].push_back(obstacle.modulate_vector(0));
        vector_item_array[i][3].push_back(obstacle.modulate_vector(1));
      }
    }
  }
  /*step03->artist实时数据更新并绘制*/
  vector<string> quiver_colors = {"lightcyan", "green", "r"};
  vector<float> quiver_width = {0.004f, 0.002f, 0.006f};
  vector<int> scales = {10, 10, 10};
  for (int i = 0; i < obs_vector_artist.size(); i++) {
    // if (i == 1) continue;
    pybind11::dict Kwargs("color"_a = quiver_colors[i], "scale"_a = scales[i], "units"_a = "width", "animated"_a = true, "width"_a = quiver_width[i],
                          "headwidth"_a = 3, "headlength"_a = 5);
    obs_vector_artist[i] =
        env_axes_ptr_->quiver(Args(vector_item_array[i][0], vector_item_array[i][1], vector_item_array[i][2], vector_item_array[i][3]), Kwargs)
            .unwrap();
    env_axes_ptr_->unwrap().attr("draw_artist")(obs_vector_artist[i]);
  }
}

/**
 *@brief:运动控制状态显示
 *@params:
 *  T:figure动画周期设置
 *  robot_pose:实时定位
 *  omega:角速度
 */
void Animation::EnvironmentDisplay() {
  //频率控制
  static int64_t last_sim_time_stamp = 0;
  if (FrequencyCtrl(ENV_DURATION, last_sim_time_stamp)) return;
  if (erase_flag_) {
    env_background_ = env_base_background_;
  }
  /*****图像元素绘制*****/
  canvas_restore_region(env_figure_ptr_->unwrap(), env_background_);
  Plotline();
  PlotPointsOnPath();
  PlotTargetOutline();
  PlotRobotOutline(Scene::ENV);
  PlotSensorObs(Scene::ENV);
  //仿真环境特有显示
  if (running_mode_ == Mode::SIMULATION) {
    PlotEntityObs(Scene::ENV);
    PlotMPCHrizon();
    DynSysAvoidDisplay();
  }
  SetEnvAxisLimit(trajectory_, 1, 2.f);
  canvas_update_flush_events(env_figure_ptr_->unwrap());
  // 背景更新
  if (!axis_change_ && blade_ && !erase_flag_) {
    canvas_restore_region(env_figure_ptr_->unwrap(), env_background_);
    // PlotBladeShadow(ax, robot_pose);
    env_background_ = canvas_copy_from_bbox(env_figure_ptr_->unwrap());
  } else {
    env_background_ = env_base_background_;
  }
}

void Animation::CmdMonitor(int buffer_length) {
  static bool once_flag = true;
  /******动画频率设置******/
  static float duration_time = 0.0f;
  static int64_t last_sim_time_stamp = 0;
  int64_t current_time_stamp = TimeToolKit::TimeSpecSysCurrentMs();
  // 时间控制
  int record_time = current_time_stamp - last_sim_time_stamp;
  if (record_time < DURATION && last_sim_time_stamp != 0) {
    return;
  }
  if (last_sim_time_stamp == 0) {
    duration_time = 0.0f;
  } else {
    duration_time += record_time / 1000.f;
  }
  last_sim_time_stamp = current_time_stamp;
  canvas_restore_region(cmd_figure_ptr_->unwrap(), cmd_background_);
  /******数据计算******/
  /*step01->实时数据更新*/
  static vector<float> time_array;
  time_array.push_back(duration_time);
  static mesh2D line_data(2);
  static vector<py::object> lines_artist(2);
  line_data[0].push_back(cmd_.linear);
  line_data[1].push_back(cmd_.angular);
  // 擦除操作
  if (erase_flag_) {
    time_array.clear();
    for (auto& line : line_data) {
      line.clear();
    }
  }
  // 数据更新
  if (time_array.size() > buffer_length) {
    time_array.erase(time_array.begin());
    line_data[0].erase(line_data[0].begin());
    line_data[1].erase(line_data[1].begin());
  }
  /*step02->static artist生成*/
  static vector<string> colors = {"r", "b"};
  if (once_flag) {
    once_flag = false;
    for (int i = 0; i < line_data.size(); i++) {
      lines_artist[i] = cmd_axes_ptr_->plot(Args(time_array, line_data[i]), Kwargs("c"_a = colors[i], "lw"_a = 1.0)).unwrap().cast<py::list>()[0];
    }
  }
  /*step03->artist实时数据更新并绘制*/
  for (int j = 0; j < lines_artist.size(); j++) {
    lines_artist[j].attr("set_data")(time_array, line_data[j]);
    cmd_axes_ptr_->unwrap().attr("draw_artist")(lines_artist[j]);
  }
  /******axis计算******/
  auto axes_xlim = cmd_axes_ptr_->get_xlim();
  if (time_array.back() > get<1>(axes_xlim) - 10) {
    float x_min = get<1>(axes_xlim) - 20.f;
    float x_max = x_min + CMD_X_RANGE;
    cmd_axes_ptr_->set_xlim(Args(x_min, x_max));
  }
  canvas_update_flush_events(cmd_figure_ptr_->unwrap());
}

void Animation::MapDisplay() {
  if (!map_init_flag_) return;
  //频率控制
  static int64_t last_sim_time_stamp = 0;
  if (FrequencyCtrl(DURATION, last_sim_time_stamp)) return;
  canvas_restore_region(map_figure_ptr_->unwrap(), map_background_);
  canvas_update_flush_events(map_figure_ptr_->unwrap());
}

void Animation::ObsMonitor() {
  //频率控制
  static int64_t last_sim_time_stamp = 0;
  if (FrequencyCtrl(DURATION, last_sim_time_stamp)) return;
  canvas_restore_region(sensor_figure_ptr_->unwrap(), sensor_background_);
  PlotRobotOutline(Scene::SENSOR);
  PlotVisionBoundary();
  PlotSensorObs(Scene::SENSOR);
  PlotGridMapObs();
  if (running_mode_ == Mode::SIMULATION) {
    PlotLidarData();
    PlotEntityObs(Scene::SENSOR);
  }
  canvas_update_flush_events(sensor_figure_ptr_->unwrap());
}

void Animation::SpaceRobot(const float& rate) {
  /*step01->频率控制*/
  static int64_t last_sim_time_stamp = 0;
  if (FrequencyCtrl(ENV_DURATION, last_sim_time_stamp)) return;
  static bool once_flag = true;
  /*step02->实时数据更新*/
  int contourN = robot::contours.size();
  static vector<py::object> contours_artists(contourN);
  vector<mesh2D> contours(contourN);
  pybind11::dict robotkwargs("c"_a = "b", "ls"_a = "-", "lw"_a = 3, "animated"_a = true);
  pybind11::dict wheelkwargs("c"_a = "k", "ls"_a = "-", "lw"_a = 3, "animated"_a = true);
  //轮廓数据空间变换
  Eigen::Matrix3f R;  // 空间旋转矩阵
  R = Eigen::AngleAxisf(robot_pose_.theta, Eigen::Vector3f::UnitZ()) * Eigen::AngleAxisf(rate * robot_pose_.pitch, Eigen::Vector3f::UnitY()) *
      Eigen::AngleAxisf(rate * robot_pose_.roll, Eigen::Vector3f::UnitX());
  for (int t = 0; t < contourN; ++t) {
    auto contour = robot::contours[t];
    mesh2D trans_contour(3);
    for (int i = 0; i < contour[0].size(); i++) {
      Eigen::Vector3f point(contour[0][i], contour[1][i], contour[2][i]);
      Eigen::Vector3f rotate_point = R * point;
      for (int j = 0; j < 3; j++) {
        trans_contour[j].push_back(rotate_point(j));
      }
    }
    contours[t] = trans_contour;
  }
  //标注数据
  static py::object text_artist;
  ostringstream oss_pitch, oss_roll;
  oss_pitch << fixed << setprecision(2) << robot_pose_.pitch * 180 / M_PI;
  oss_roll << fixed << setprecision(2) << robot_pose_.roll * 180 / M_PI;
  string attitude = "pitch: " + oss_pitch.str() + " ; roll: " + oss_roll.str();
  //设置图像属性
  if (once_flag) {
    //文字
    once_flag = false;
    py::object trans_figure = space_figure_ptr_->unwrap().attr("transFigure");
    text_artist = space_axes_ptr_->text(Args(1.5, 1, 1.5, attitude), Kwargs("transform"_a = trans_figure, "fontsize"_a = 15)).unwrap();
    //图层
    for (int i = 0; i < contourN; i++) {
      auto kwargs = i < 1 ? robotkwargs : wheelkwargs;
      contours_artists[i] = space_axes_ptr_->plot(Args(contours[i][0], contours[i][1], contours[i][2]), kwargs).unwrap().cast<py::list>()[0];
    }
  }
  /*step03->动画更新*/
  canvas_restore_region(space_figure_ptr_->unwrap(), space_background_);
  // text数据
  text_artist.attr("set_text")(attitude);
  space_axes_ptr_->unwrap().attr("draw_artist")(text_artist);
  // contoours
  for (int i = 0; i < contourN; i++) {
    contours_artists[i].attr("set_data")(contours[i][0], contours[i][1]);
    contours_artists[i].attr("set_3d_properties")(contours[i][2]);
    space_axes_ptr_->unwrap().attr("draw_artist")(contours_artists[i]);
  }
  canvas_update_flush_events(space_figure_ptr_->unwrap());
}

/**
 *@brief:障碍物速度规划显示
 *@params:
 * T:figure动画周期设置
 */
void Animation::SurfaceVelPlanning(int T) {
  //画框基础参数设置
  static int fig_width = 5;
  static int ratio = 1;
  /******动画频率设置******/
  static int64_t last_sim_time_stamp = 0;
  if (FrequencyCtrl(T, last_sim_time_stamp)) return;
  /*****figure图框设置*****/
  static auto plt = matplotlibcpp17::pyplot::import();
  matplotlibcpp17::mplot3d::import();
  static auto fig = plt.figure(Args(), Kwargs("figsize"_a = py::make_tuple(ratio * fig_width, fig_width), "dpi"_a = 100, "tight_layout"_a = true,
                                              "facecolor"_a = "lightgray"));
  static auto ax = fig.add_subplot(Args(), Kwargs("projection"_a = "3d"));
  static py::object background;
  static bool once_flag = true;
  static py::object point_artist;
  static py::object scatter;
  auto V = DC_Instance->GetAvoidPlanningVel();
  //设置图像属性
  if (once_flag) {
    once_flag = false;
    plt.show(Args(), Kwargs("block"_a = 0));
    auto x_array = mathTools::linspace(0.0, 1.0, 100);
    auto y_array = mathTools::linspace(0.0, 1.0, 100);
    auto [X, Y] = mathTools::meshgrid(x_array, y_array);
    auto Z = mathTools::computeZ(X, Y);
    // to numpy array (vector<vector> is converted to list of list)
    const auto X_ = py::array(py::cast(std::move(X)));
    const auto Y_ = py::array(py::cast(std::move(Y)));
    const auto Z_ = py::array(py::cast(std::move(Z)));
    const auto surf =
        ax.plot_surface(Args(X_, Y_, Z_), Kwargs("cmap"_a = cm::coolwarm, "edgecolor"_a = "k", "linewidth"_a = 1, "animated"_a = false));

    //速度规划点
    pybind11::dict kwargs("c"_a = "lawngreen", "ls"_a = "None", "animated"_a = true, "marker"_a = "o", "markersize"_a = 10);
    point_artist = ax.plot(Args(V(0), V(1), V(2)), kwargs).unwrap().cast<py::list>()[0];
    // // ax.set_axis_off();
    ax.set_xlim(Args(0.0f, 1.0f));
    ax.set_ylim(Args(0.0f, 1.0f));
    ax.set_zlim(Args(-0.01, 1.01));
    plt.pause(Args(0.1));
    background = canvas_copy_from_bbox(fig.unwrap());
  }
  /*****图像元素绘制*****/
  canvas_restore_region(fig.unwrap(), background);
  point_artist.attr("set_data")(V(0), V(1));
  point_artist.attr("set_3d_properties")(V(2));
  ax.unwrap().attr("draw_artist")(point_artist);
  ax.grid(Args(true));
  canvas_update_flush_events(fig.unwrap());
}

void Animation::AnimationLoop() {
  GetCommonData();
  EnvironmentDisplay();
  CmdMonitor(600);
  MapDisplay();
  ObsMonitor();
  SpaceRobot(1.5f);
  SurfaceVelPlanning(DURATION);
}

}  // namespace animation
}  // namespace modules
