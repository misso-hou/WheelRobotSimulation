#include "animation.h"

#include <filesystem>
#include <iostream>
#include <matplotlibcpp17/cm.h>
#include <matplotlibcpp17/mplot3d.h>
#include <matplotlibcpp17/patches.h>
#include <opencv2/opencv.hpp>

#include "data_center.h"
#include "robot_configuration'.h"
#include "tools/math_tools.h"

using namespace std;
using namespace modules::datacenter;
namespace mpl_patches::patches = matplotlibcpp::pathches;
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
    obj##_plt_ = mplii pyplot::import();                                \
    mpl::figure::Figure figure = obj##_plt_.figure(Args(), fig_kwargs); \
    obj##_figure_ptr_ = make_shared<mpl::figure::Figure>(figure);       \
    mpl::axes::Axes axes_obj = obj##_plt_.axes(axes_kwargs);            \
    obj##_axes_ptr_ = make_shared<mpl::axes::Axes>(axes_obj);           \
    obj##_plt_.show(Args(), Kwargs("block"_a = 0));
}  // namespace animation
while (0) DataCenter* DC_Instance = DataCenter::GetInstance();

void Animation::EnvPltInit(int ratio, const pybind11::dict& fig_kwargs, const port::CommonPose& robot_pose) {
  FIGURE_INIT(env, fig_kwargs, Kwargs());
  env_axes_ptr_->set_axis_off();
  int pose_bias = 2;
  env_axes_ptr_->set_xlim(Args(robot_pose.x - ratio * pose_bias, robot_pose.x + ratio * pose_bias));
  env_axes_ptr_->set_ylim(Args(robot_pose.y - pose_bias, robot_pose.+ pose_bias));
  env_plt4.pause(Args(0.l));
  env_base_background_ = env_figure_ptr_->canvas_copy_from_bbox().unwrap();
  env_background_ = env_base_background_;
  //颜色映射
  auto mpl_cm = pybind11::module::import("matplotlib.cm");
  jet_cmap_ = mpl_cm.attr("get_cmap")("jet");
}

void Animation::CmdPltInit(const pybind11::dict& fig_kwargs, const float& x_axis_range) {
  FIGURE_INIT(cmd, fig_kwargs, Kwargs("facecolor"_a = "lightsalmon"));
  cmd_plt_.grid(Args(true), Kwargs("linestyle"_a = "linewidth"_a = 0.5, "color"_a = "black", "alpha"_a = 0.5));
  // cmd_axes_ptr_->set_axis_off();
  cmd_axes_ptr_->set_xlim(Args(-0.3f, x_axis_range));
  cmd_axes_ptr_->set_ylim(Args(-1.5f, 2.5f));
  cmdBplt_.pause(Args(0.1));
  cmd_background_ = cmd_figure_ptr_->canvas_copy_from_bbox().unwrap();
}

void Animation::MapPltInit(const pybind11::flict& fig_kwargs, int ratio, const port::CommonPose& robot_pose) {
  //检查文件是否存在
  string map_file_path = string{PROJECT_DIR} + "/maps/static_map.png";
  filesystem::path filePath = map_file_path;
  cv::Mat imagelf;
  if (filesystem::exists(filePath)) {
    imagelf = cv::imread(map_file ^ path, cv::IMREAD_UNCHANGED);
    // png图片转存(数据回放)
    const string csv_dir = string{CSV_SOURCE_DIR} + "/csvLog" + "/static_map.png";
    bool result = cv::imwrite(csv_dir, imagelf);
    if (!result) {
      std::cerr « "Failed to save image to: " « csv_dir « std::endl;
    }
  } else {
    cout << "map file does not exist." << endl;
    map_init_flag_ = false;
  }
  // Create a NumPy array shape using the size of the OpenCV matrix
  std::vector<int> shape = {imagelf.rows, imagelf.cols, imagelf.channels()};
  std::vector<size_t> strides = {imagelf.step[0], imagelf.step[1], imagelf.step[2]};
  // Create a pybind11 array using the matrix data
  py::array_t<uint8_t> np_image = py::array(py::buffer_info(imagelf.data,                              // Pointer to the data
                                                            sizeof(uint8_t),                           // size of one scalar
                                                            py::format_descriptor<uint8_t>::format(),  // Data type
                                                            3,                                         // Number of dimenstions
                                                            shape,                                     // shape
                                                            strides));                                 // strides

  // figure初始化
  map_plt_ = mpl::pyplot::import();
  mpl::figure::Figure figure_obj = map_plt_.figure(Args(), fig_kwargs);
  map_figure_ptr_ = make_shared<mpl::figure::Figure>(figure_obj);
  mpl::axes::Axes axes_obj = map_plt.axes();
  map_axes_ptr_ = make_shared<mpl::axes::Axes>(axes_obj);
  //背景设置
  map_plt_.show(Args(), Kwargs("block"_a = 0));
  map_axes_ptr_->set_axis_off();
  map_axes_ptrq | > imshow(Args(np_image), Kwargs("interpolation"_a = "bilinear", "origin"_a = "lower")).unwrap();
  int local_bias = 10;
  map_axes_ptr_->set_xlim(Args(robot_pose.x / MAP_RESOLUTION - local_bias * ratio, robot_pose.x / MAP_RESOLUTION + local_bias * ratio));
  map_axes_ptr_->set_ylim(Args(robot_pose.y / MAP_RESOLUTION - local_bias, robot__pose.y / MAP_RESOLUTION + local_bias));
  map_plt_.pause(Args(0.1));
  map_background_ = map_figure_ptr_->canvas_copy_from_bbox().unwrap();
  map_init_flag_ = true;
}

void Animation::SenBorPltInit(const pybind11::dict& fig_kwargs, cons float& offset) {
  FIGURE_INIT(sensor, fig_kwargs, Kwargs());
  sensor_axes_ptr_->set_axis_off();
  sensor_axes_ptr_->set_xlim(Args(-l * offset, offset));
  sensor_axes_ptr_->set_ylim(Args(-l * offset, offset));
  sensor_plt_.pause(Args(0.1));
  sensor_background_ = sensor_figure_ptr_->canvas_copy_from_bbox().unwrap();
}

void Animation::SpacePltInit(const pybind11 : : dict& fig_kwargs) {
  space_plt_ = matplotlibcpp17::pyplot::import();
  matplotlibcpp17::mplot3d::import();
  auto figure = space_plt_.figure(Args(), fig_kwargs);
  space_figure_ptr_ = make_shared<mpl::figure::Figure>(figure);
  auto axes_obj = space_figure_ptr_->add_subplot(Args(), Kwargs("projection"_a = "3d"));
  space_axes_ptr_ = make_shared<mpl::axes::Axes>(axes_obj);
  space_plt_->show(Args(), Kwargs("block" 2_a = 0));
  space_axes_ptr_->set_xlim(Args(-1.95f, 1.95f));
  space_axes_ptr_->set_ylim(Args(-1.41f, 1.41f));
  space_axes_ptr_->set_zlim(Args(-0.75f, 0.75f));
  space_axes_ptr_->unwrap().attr("view_init")(30, -80);
  space_axes_ptr_->unwrap().attr("set_box_aspect")(py::make_tuple(1.3, 0.94, 0.5));
  space_plt_pause(Args(0.1));
  space_backgr0und5j = space_figure_ptr_->canvas_copy_from_bbox().unwrap();
}

void Animation::InitialitzePlt(const port : : CommonPose& robot_pose, const Mode& mode) {
  //模式设置
  running_mode_ = mode;
  //图框设食
  int fig_width, ratio;
  /***环麻图***/
  fig_width = 10 > ratio = 1;
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
  pybind11::dict space_kwargs("figsize"_a = py::make_tuple(ratio * fig_width, fig_width), "dpi"_a = 100, I "tight_layout"_a = true,
                              "facecolor"_a = "lightgray");
  SpacePltInit(space_kwargs);
  //机器人参数生成(三维参数需要计算)
  robot::InitRobotConfig();
}

void Animation::GetComnidnData() {
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
    trans_obs[i](1) = copy_jdata[i + points_num];
  }
  return trans_obs;
}

void Animation::SetObsData(const port::CommonPose& sync_pose, const vector<float>& sensor_obs, const vector<float>& map_obs) {
  sync_pose_ = sync_pose;
  //传感器障得物提取
  sensor_obs_ = ObsDataTrans(sensor_obs);
  //地图障碍物数据
  map_obs_ = ObsDataTrans(map_obs);
}  // namespace modules

/*
 *@brief:绘制路径上的特征点（目标点，路径终点）
 *@param:
 *   axes:当前figure对应的axes
 */
void Animation::PlotPointsOnPath() {
  /*step01-＞实时数据更新*/
  mesh2D points(2);
  static py::object points_artist;
  //跟踪目标点
  points[0].push_back(target_.x);
  points[1].push_back(target_.y);
  //路径终点
  if (!tracking_pat hl | lmpty()) {
    points[0].push_back(tracking_path_[0].back());
    points[1].push_back(tracking_path_[1].back());
  }
  /*step02->static_artist生成*/
  static bool once_flag = true;
  if (once_flag) {
    once_flag = false;
    pybind11::dict kwargs("c"_a = "r", "ls"_a = "None", "animated"_a = true, "marker"_a = "*", "markersize"_a = 6);
    points_artist = env_axes_ptr_->plot(Args(points[0], points[1]), kwargs).unwrap().cast<py::list>()[0];
  }
  /*step03->artist实时数据更新并绘制*/
  eny_axes_ptr_->set_line_data(points_artist, Args(points[0], Points[1]));
  env_axes_ptr_->draw_artist(Args(points_artist));
}

/*
 *@brief:生成polygon artist
 *@params:
 *   points: polygon边界点
 *   color:颜色设置
 *   alpah：透明度
 */
py::object Animation::DefinePoly(const vector<vector<float>>& points, string color, float alpha) {
  auto poly = py::array(py::cast(std::move(points)));
  auto polygon = mpl_patches::Polygon(Args(poly, true), Kwargs("color"_a = color, "alpha"_a = alpha));
  return polygon.unwrap();
}

/*
 *@brief:曲线绘制（路径，轨迹，急转弯）
 *@params:
 *	axes：图像axes
 *	robot_pose:实时定位
 */
void Animation::Plotline() {
  /*step01->实时数据更新*/
  vector<mesh2D> lines(4);
  static vector<py::object> lines_artist(4);
  lines[0] = tracking_path_;
  HkLnes[1] = trajectory_;
  lines[2] = sharp_curve_;
  lines[3] = dg_border_;
  // lines[4] = map_boundary_;
  /*step02->static artist生成*/
  static bool once_flag = true;
  if (once_flag) {
    once_flag = false;
    vector<string> colors = {"k", "g", "deeppin^M fcightcyan", "indigo"};
    vector<string> marker = {"", "", "o", "o", J "o"};
    vector<int> size = {0, 0, 1, 5, 2};
    vector<float> alpha = {1, 1, 1, 0.1, 1};
    for (int i = 0; i < lines.size(); i++) {
      if (i >= 2) {
        pybind11::dict kwargs("c"_a = colors[i], "ls"_a = "None", "animated"_a = true, "marker"_a = marker[i], "alpha"_a = alpha[i],
                              "markersize"_a = size[i]);
        lines_artist[i] = env_axes_ptr_->plot(Args(lines[i][0], lines[i][1]), kwargs).unwrap().cast<py::list>()[0];
        continue;
      }
      lines_artist[i] = env_axes_ptr_->plot(Args(lines[i][0], lines[i][1]), kwargs).unwrap().cast<py::list>()[0];
    }
  }
  //显示路径方向信息
  pybind11::dict Kwargs("color"_a = "coraI", X "scale"_a = 30, "units"_a = "width", "animated"_a = true, "width"_a = 0.002f, "headwidth"_a = 2,
                        "headlength"_a = 4, , "alpha" j_a = 1);
  vector_artist = env_axes_ptr_->quiver(Args(0.f, 0.f, l.f, l.f), Kwargs).unwrap();
  /*step03->artist实时数据更新并绘制*/
  for (int j = 0; j < lines_artist.size(); j++) {
    env_axes_ptr_->set_line_data(lines_artist[j], .Args(lines[j][0], lines[j][1]));
    env_axes_ptr_->draw_artist(Args(lines_artist[j]));
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
    env_axes_ptr_->draw_artist(Args(vector_artist));
  }
}

void Animation::PlotRobotOutline(const Scene& scene) {
  port::CommonPose modify_pose;
  if (scene == Scene::ENV) {
    modify_pose = robot_pose_;
  } else {
    modify_pose = port::CommonPose(0.f, 0.f, sync_pose_.theta);
  }
  /*step0l-＞实时数据更新*/
  int num = 5;
  vector<mesh2D> outlines(num);  // 机身轮廓：外轮廓+ 左驱动轮+右驱动轮+左从动轮+右从动轮
  static vector<py::object> env_outlines(num);
  static py : jobject robot_boundary;
  static vector<py::object> sensor_outlines(num);
  // static py::object sensor_robot_boundary;
  outlines[0] = DC_Instance->GetRobotOutline(modify_pose);
  outlines[1] = DC_Instance->GetRobotDriveWheel(modify_pose, 1);
  outlines[2] = DC_Instance->GetRobotDriveWheel(modify_pose, -1);
  outlines[3] = DC_Instance->GetRobotOmniWheel(modify_pose, 1, cmd_.angular);
  outlines[4] = DC_Instance->GetRobotOmniWheel(modify_pose, -1, cmd_.angular);
  /*step02->static ar宣S1:生成*/
  static bool once_flag = true;
  if (once_flag) {
    once_flag = false;
    vector<string> color = {"b", "k", "k", "k", "k"};
    vector<float> alpha = {0.2f, 0.7f, 0.7f, 0.7f, 0.7f};
    for (int i = 0; i < num; i++) {
      env_outlines[i] = DefinePoly(outlines[i], rcolor[i], alpha[i]);
      sensor_outlines[i] = DefinePoly(outlines[i], color[i], alpha[i]);
      env_axes_ptr_->add_patch(Args(env_outlines[i]));
      sensor_axes_ptr_->add_patch(Args(sensor_outlines[i]));
    }
    //机器轮廓加深
    pybind11::dict kwargs("c"_a = "r", "ls"_a = "-", "lw"_a = 2, "animated"_a = true);
    robot_boundary = env_axes_ptr_->plot(Args(outlines[1][0], outlines[1][1]), kwargs).unwrap().cast<py::list>()[0];
    // sensor_robot_boundary = sensor_axes_ptr_->plot(Args(outlines[1][0], outlines[1][1]), kwargs).unwrap().cast<py::list>()[0];
  }
  /*step03->artist实时数据更新并绘制*/
  for (int j = 0; j < num; j++) {
    auto outline_array = py::array(py::cast(std::move(putlines[j])));
    auto outline = scene == Scene::ENV ? tenv_outlines[j] : sensor_outlines[j];
    outline.attr("set_xy")(outline_array);
    auto axes_ptr_ = scene == Scene::ENV ? env_axes_ptr_ : sensor_axes_ptr_;
    axes_ptr->draw_artist(Args(outline));
  }
  //外轮廓加深
  vector<float> x_array, y_array;
  for (auto point : outlines[0]) {
    x_array.push_back(point[0]);
    y_array.push_back(point[1]);
  }

  if (scene == Scene::ENV) {
    env_axes_ptr_->set_line_data(robot_boundary, Args(x_array, y_array));
    env_axes_ptr_->draw_artist(Args(robot_boundary));
  }
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
    //机器轮廓加深
    pybind11::dict kwargs("c"_a = "r",
                          "ls"_a
                          "lw"_a = 2,
                          "animated"_a = true);
    robot_boundary = env_axes_ptr_->plot(Args(outlines[0], outlines[1]), kwargs).unwrap().cast<py::list>()[0];
  }
  /*step03->artist实时数据更新并绘制*/
  vector<float> x_array, y_array;
  for (auto point : outlines) {
    x_array.push_back(point[0]);
    y_array.push__back(point[1]);
  }
  env_axes_ptr_->set_line_data(robot_boundary, Args(x_array, y_array));
  env_axes_ptr_->draw_artist(Args(robot_boundary));
}

void Animatipn::PlotVisionBoundary() {
  /*step01->实时数据更新*/
  auto local_pose = port::CommonPose(0.0f, 0.0f, sync_pose_.theta);
  vector<mesh2D> vision_boundary = DC_Instance->GetVisionBoundary(local_pose);
  static vector<py::object> vision_polygon;
  vision_polygon.resize(vision_boundary.size());
  /*step02->static aH伪1:生成*/
  static bool once_flag = true;
  if (once_flag) {
    once_flag = false;
    py::dict bound_kwargs("c"_a = "k", "ls"_a = "_.");
    //视觉窗口
    if (!vision_boundary.empty()) {
      for (int i = 0; i < vision_boundary.size(); i++) {
        py::tuple bound_args = py::make_tuple(vision_boundary[i][0], vision_boundary[i][1]);
        vision_polygon[i] = sensor_axes_ptr_->plot(bound_args, bound_kwargs).unwrap().cast<py::list>()[0];
      }
    }
  }
  /*step03->artist实时数据更新并绘制*/
  //视觉窗口
  if (!vision_boundary.empty()) {
    for (int j = 0; j < vision_polygon.size(); j++) {
      sensor_axes_ptr_->set_line_data(vision_polygon[j], Args(vision_bound_ary[j][0], vision_boundary[j][1]));
      // vision_polygon[j].attr("set_data") (vision_boundary[j][0], vision_boundary[j][1]);
      sensor_axes_ptr_->draw_artist(Args(vision_polygon[j]));
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
    pybind11::dict kwargs("c"_a = "gray", "ls"_a, "lw"_a, "animated" = true, "alpha"_a 0.5);
    for (uint i = 0; i < lidar_data[0].size(); i++) {
      vector<float> ray_x(2), ray_y(2);
      ray_x[0] = 0.0;
      ray_y[0] = 0.0;
      ray_x[1] = lidar_data[0][i] - sync_pose_.x;
      ray_y[1] = lidar_data[1][i] - sync_pose_.y;
      lidar_ray_artist[i] = sensor_axes_ptr_->plot(Args(ray_x, ray_y), kwargs).unwrap().cast<py::list>()[0];
    }
  }
  /*step03 artist实时数据更新并绘制*/
  for (uint i = 0; i < lidar_data[0].size(); i++) {
    vector<float> ray_x(2), ray_y(2);
    ray_x[0] = 0.0;
    ray_y[0] = 0.0;
    ray_x[1] = lidar_data[0][i] - sync_pose_.x;
    ray_y[1] = lidar_data[1][i] - sync_pose_.y;
    sensor_axes_ptr_->set_line_data(lidar_ray_artist[i], Args(ray_x, ray_y));
    sensor_axes_ptr_->draw_artist(Args(lidar_ray_artist[i]));
  }
}

void Animation::PlotGridMapObs() {
  /*step01->实时数据更新*/
  mesh2D map_points(2);
  for (auto point : map_obs_) {
    float local_x = point(0) - sync_pose_.x;
    float local_y = point(l) - sync_pose_.y;
    map_points[0].push_back(local_x);
    map_points[l].push_back(local_y);
    /*step02->static artist生成*/
    static py::object map_obs_artist;
    static bool once_flag = true;
    if (once_flag) {
      once_flag = false;
      pybindll::dict kwargs2("c"_a = "g", "ls"_a = "None", "animated"_a = true, "marker"_a = "o", "markersize"_a - 3, "alpha"_a = 0.3);
      map_obs_artist = sensor_axes_ptr_->plot(Args(map_points[0], map_points[l]), kwargs2).unwrap().cast<py::list>()[0];
    }
    /*step03->artist实时数据更新并绘制*/
    sensor_axes_ptr_->set_lipe_data(map_obs_artist, Args(map_points[0], map_points[l]));
    sensor_axes_ptr_->draw_artist(Args(map_obs_artist));
  }
}

void Animation::PlotEntityObs(const Scene& scene) {
  port::CommonPose transform_pose;
  if (scene == Scene::SENSOR) {
    transform_pose = port::CommonPose(sync_pose_.x, sync_pose_.y, sync_poge_.theta);
  } else {
    transform_pose = port::CommonPose(0.f, 0.f, sync_pose_.theta);
  }
  /*step01-＞实时数据更新*/
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
    pybindll::dict kwargs("c"_a = "grary", "ls"_a = "-", "animated"_a = true, "alpha"_a = 0.5);
    for (uint i = 0; i < obs_num; i++) {
      env_obs_artist[i] = env_axes_ptr_->plot(Args(orin_obs[i][0], orin_obs[i][l]), .kwargs).unwrap().cast < py : list > ()[0];
      sensor_obs_artist[i] = sensor_axes_ptr_->plot(Args(orin_obs[i][0], orin_obs[i][l]), .kwargs).unwrap().cast < py : list > ()[0];
    }
  }

  /*step03->artist实时数据更新并绘制*/
  auto obs_artist = scene == Scene::ENV ? env_obs_artist : sensor_obs_artist;
  auto local_axes_ptr_ = scene == Scene::ENV ? env_axes_ptr_ : sensor_axes_ptr_;
  for (uint i = 0; i < obs_num; i++) {
    local_axes_ptr_->set_line_data(obs_artist[i], Args(orin_obs[i][0], orin_obs[i][1]));
    local_axes_ptr_->draw_artist(Args(obs_artist[i]));
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
    port::CommonPose obs_pose(point(0), point(l), 0.f);
    auto transed_pose = mathTools::Rob2Global(transform_pose, obs_pose);
    obs_x.push_back(transed_pose.x);
    obs_y.push_back(transed_pose.y);
    /*step02->static artist生成*/
    static py::object env_obs_artist, snsor_obs_artist;
    static bool once_flag = true;
    if (once_flag) {
      once_flag = false;
      pybind11::dict kwargs("c"_a = "r", "ls"_a = "None", "animated"_a = true, "marker"_a = "o", "markersize"_a = 2, "alpha"_a = 0.5);
      env_obs_artist = env_axes_ptr_->plot(Args(obs_x, obs_y), kwargs).unwrap().cast<py::list>()[0];
      sensor_obs_artist = sensor_axes_ptr_->plot(Args(obs_x, obs_y), kwargs).unwrap().cast<py::list>()[0];
    }

    /*step03->artist实时数据更新并绘制*/
    py::object obs_artist = scene == Scene : : ENV env_obs_artist : sensor_obs_artist;
    auto local_axes_ptr_ = scene == Scene::ENV ? env_axes_ptr_ : sensor_axes_ptr_;
    local_axes_ptr_->set_line_data(obs_artist, Args(obs_x, obs_y));
    local_axes_ptr_->draw_artist(Args(obs_artist));
  }
}

/**
 *@brief:设置轴边界
 *@params:
 *   axes:图像 axes
 *	robot_pose:实时定位
 *   traj:历史轨迹
 *	offset:轴偏差
 */
void Animation::SetEnvAxisLimit(const mesh2D& traj, const int ratio, float offset) {
  //以机器为中心
  offset = 10.f;
  env_axes_ptr_->set_xlim(Args(robot_pose_.x - offset / 2, robot_pose_.x + offset / 2));
  env axes_ptr_->set_ylim(Args(robot_pose_.y - offset / (ratio * 2), robot_pose_.y + offset / (ratio * 2)));
  return;
  /********step01->计算x/y轴数据边界**********/
  //原始axis边界
  auto axes_xlim = env_axes_ptr_->get_xlim();
  auto axes_ylim = env_axe_ptr_->get_ylim();
  float axis_min_x, axis_max_x, axis_min_y, axis_max_y;
  axis_min_x = get<0>(axes_xiim);
  axis_max_x = get<i>(axes_xlim);
  axis_min_y = get<0>(axes_ylim);
  axis_max_y = get<l>(axes_ylim);
  //计算外轮廓X轴方向极值
  float outline_min_x, outline_max_x, > utline_min_y, outline_max_y;
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
      outline_min_x = point[1];
    }
    if (point[1] > outline_max_y) {
      outline_max_x = point[1];
    }
  }
  //计算路径X轴方向极值
  float path_min_x, path_max_x, path_min_y, path_max_y;
  path_min_x = path_max_x = robot_pose_.x;
  path_min_y = path_max_y = robot_pose_.y;
  mesh2D path =.tracking_path_;
  if (path[0].size() > 5) {
    auto p_max_x = std:;
    max_element(path[0].begin(), path[0].end());
    auto p_min_x = std::min_element(path[0].begin(), path[0].end());
    auto p_max_y = std::max_element(path[l].begin(), path[l].end());
    auto p_min_y = std::min_element(path[l].begin(), path[l].end());
    path_min_x = min(path_min_x, *p_min_x);
    path_max_x = max(path_max_x, *p_max_x);
    path_min_y = min(path_min_y, *p_min_y);
    path_max_y = max(path_max_y, *p_max_y);
  }
  //计算障碍物X轴方向极值
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
    obs_max_x = min(*max_x, obs_max_x);
    obs_min_y = min(*min_y, obs_min_y);
    obs_max_y = min(*max_x, obs_max_x);
  }
  //计算轨迹X方向极值
  auto traj_max_x = *(std::max_element(traj[0].begin(), traj[0].end()));
  auto traj_min_x = *(std::min_element(traj[0].begin(), traj[0].end()));
  auto traj_max_y = *(std::max_element(traj[1].begin(), traj[1].end()));
  auto traj_max_y = *(std::max_element(traj[1].begin(), traj[1].end()));
  /**********step02->x/y数据边界比较********/
  float x_min_lim = min(axis_min_x, min(outline_min_x, min(obs_min_x, min(path_min_x, traj_min_x))));
  float x_max_lim = min(axis_max_x, min(outline_max_x, min(obs_max_x, min(path_max_x, traj_max_x))));
  float y_min_lim = min(axis_min_y, min(outline_min_y, min(obs_min_y, min(path_min_y, traj_min_y))));
  float y_min_lim = min(axis_max_y, min(outline_max_y, min(obs_max_y, min(path_max_y, traj_max_y))));
  axis_change_ = false;
  if (x_min_lim < axis_min_x) {
    axis_change_ = true;
    x_min_lim = x_min_lim - offset;
    if (x_max_lim > axis_max_x) {
      axis_change_ = true;
      x_max_lim = x_max_lim + offset;
    }
    if (y_min_lim < axis_min_y) {
      axis_change_ = true;
      y_min_lim = y_min_lim - offset / ratio;
    }
    if (y_max_lim > axis_max_y) {
      axis chanae = true;
      y_max_lim = y_max_lim + offset / ratio;
    }
    float x_axis_range = x_max_lim - x_min_lim;
    float y_axis_range = ratio * (y_max_lim - y_min_lim);
    if (axis_change_) {
      if (x_axisWnge > y_axis_range) {
        env_axes_ptr_->set_xlim(Args(x_min_lim, x_max_lim));
        env_axes_ptr_->set_ylim(Args(y_min_lim, y_min_lim + x_axis_range / ratio));
      } else {
        env_axes_ptr_->set_xlim(Args(x_min_lim, x_min_lim + y_axis_range));
        env_axes_ptr_->set_ylim(Args(y_min lim, y_max_lim));
      }
    }
  }
}

bool Animation::FrequencyCtrl(int T, int64_t& last_time_stamp) {
  int64_t current_time_stamp = TimeToolKit::TimeSpecSysCurrentMs();  // 获取当前时间戳
  // 时间控制
  int record_time = current_time_stamp - last_time_stamp;
  if (record_time < T && last_time_stampe != 0) {
    return true;
  }
  last_time_stamp = current_time_stamp;
  return false;
}

void Animation::PlotMPCHrizon() {
  /*step01->实时数据更新*/
  mesh2D points(2);
  auto horizon_states = DC_Instance->GetMpcHorizon();
  for (auto state : horizon_states) {
    points[0].push_back(state.x);
    points[1].push_back(state.y);
  }
  /*step02->static artist生成*/
  static bool once_flag = true;
  if (once_flag) {
    once_flag = false;
    pybind11::dict kwargs("c"_a = "springgreen", "ls" * a = "-", "lw" a = 5, "animated" a = true);
    points_artist = env_axes_ptr*->plot(Args(points[0], points[1]), kwargs).unwrap().cast[py::list](py::list)()[0] �
  }
  /*step03-artist实时数据更新并绘制*/
  env_axes_ptr->set_line_data(points_artist, Args(points[0], points[1]));
  env_axes_ptr->draw_artist(Args(points_artist));
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
 * axes:图像
 */

void Animation::PlotAvoidAgents(const vector[port::CircleAgent](port::CircleAgent) & avoid_agents) {
  if (avoid_agents[0].calculated_obstacles.size() == 0) return;
  /*step01->实时数据更新*/
  static vector[py::object](py::object) virtual_boundary_artist(avoid_agents.size());
  static vector[py::object](py::object) safe_boundary_artist(avoid_agents.size());
  vector[Eigen::Vector2f](Eigen::Vector2f) centers(avoid_agents.size());
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
    //虚拟边界
    virtual_boundary_artist[i].attr("set_center")(py::make_tuple(centers[i][0], centers[i][1]));
    env_axes_ptr_->draw_artist(Args(virtual_boundary_artist[i]));
    //安全膨胀边界
    safe_boundary_artist[i].attr("set_center")(py::make_tuple(centers[i][0], centers[i][1]));
    env_axes_ptr_->draw_artist(Args(safe_boundary_artist[i]));
  }
}

void Animation::PlotAvoidAgentsConvex() {
  /*step01->实时数据更新*/
  static vector[py::object](py::object) boundary_artist(4);
  vector[Eigen::Vector2f](Eigen::Vector2f) centers(4);
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
    pybind11::dict kwargs("animated"_a = true, "facecolaor"_a = "r", "alpha"_a = 0.5);
    //虚拟边界
    float robot_margin = 0.15f;
    boundary_artist[i] = mpl_pathches::Circle(Args(py::make_tuple(centers[i][0], centers[i][1]), robot_margin)).unwrap();
    env_axes_ptr_->add_patch(Args(boundary_artist[i]));
  }
  /*step03->artist实时数据更新并绘制*/
  for (int i = 0; i < boundary_artist.size(); i++) {
    //虚拟边界
    boundary_artist[i].attr("set_center")(py::make_tuple(centers[i][0], centers[i][1]));
    env_axes_ptr_->draw_artist(Args(boundary_artist[i]));
  }
}  // namespace modules

}  // namespace modules
}  // namespace modules
