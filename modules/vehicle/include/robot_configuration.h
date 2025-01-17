#pragma once

#include <iostream>
#include <vector>

using namespace std;

namespace modules {
namespace vehicle {

using mesh2D = vector<vector<float>>;

// 轮廓参数定义
const float length = 1.3;
const float width = 0.94;
const float height = 0.5;
const float omni_wheel_x = 0.921;
const float omni_wheel_y = width / 2 - 0.08;
const float drive_wheel_length = 0.3f;
const float drive_wheel_width = 0.16f;
const int dw_point_num = 15;
const float omni_wheel_length = 0.2f;
const float omni_wheel_width = 0.08f;
const int omni_point_num = 10;
static mesh2D left_wheel(3);
static mesh2D right_wheel(3);
static mesh2D left_omni_wheel(3);
static mesh2D right_omni_wheel(3);
static mesh2D wireframe(3);

static vector<vector<float>> outline_points = {{-0.131f * length, width / 2},
                                               {0.869f * length, width / 2},
                                               {0.869f * length, -width / 2},
                                               {-0.131f * length, -width / 2},
                                               {-0.131f * length, width / 2}};

static vector<vector<float>> drive_wheel_points = {{-drive_wheel_length / 2, width / 2},
                                                   {drive_wheel_length / 2, width / 2},
                                                   {drive_wheel_length / 2, (width / 2 - drive_wheel_width)},
                                                   {-drive_wheel_length / 2, (width / 2 - drive_wheel_width)},
                                                   {-drive_wheel_length / 2, width / 2}};

static vector<vector<float>> omni_wheel_points = {{-omni_wheel_length / 2, omni_wheel_width / 2},
                                                  {omni_wheel_length / 2, omni_wheel_width / 2},
                                                  {omni_wheel_length / 2, -omni_wheel_width / 2},
                                                  {-omni_wheel_length / 2, -omni_wheel_width / 2},
                                                  {-omni_wheel_length / 2, omni_wheel_width / 2}};

const vector<vector<float>> front_vision_boundary = {{1.433f, 1.374f}, {3.117f, 6.f},     {8.933f, 6.f},   {8.933f, -6.f},
                                                     {3.117f, -6.f},   {1.433f, -1.374f}, {1.433f, 1.374f}};

const vector<vector<float>> back_vision_boundary = {{-0.812f, -0.47f}, {-5.362f, -4.475f}, {-5.362f, 4.475f}, {-0.812f, 0.47f}, {-0.812f, -0.47f}};

const vector<vector<float>> left_vision_boundary = {
    {-0.0682088f, 0.699578f}, {-5.19495f, 4.26254f}, {4.08814f, 6.23343f}, {0.851296f, 0.894799f}, {-0.0682088f, 0.699578f}};

const vector<vector<float>> right_vision_boundary = {
    {0.851296f, -0.926799f}, {4.08814f, -6.26543f}, {-5.19495f, -4.29454f}, {-0.0682088f, -0.731578}, {0.851296f, -0.926799f}};

const vector<vector<float>> cube_vertices = {{-0.131f * length, width / 2, height / 2},  {0.869f * length, width / 2, height / 2},
                                             {0.869f * length, -width / 2, height / 2},  {-0.131f * length, -width / 2, height / 2},
                                             {-0.131f * length, width / 2, -height / 2}, {0.869f * length, width / 2, -height / 2},
                                             {0.869f * length, -width / 2, -height / 2}, {-0.131f * length, -width / 2, -height / 2}};

const vector<vector<float>> cube_faces = {{0, 1, 2, 3, 0},                   //顶
                                          {4, 5, 6, 7, 4},                   //底
                                          {0, 1, 5, 4, 0},                   //右
                                          {3, 2, 6, 7, 3}, {1, 2, 6, 5, 1},  //前
                                          {0, 3, 7, 4, 0}};

inline void GenerateSpaceDriveWheel() {
  float angle = 2 * M_PI / dw_point_num;
  //单圈数据点
  std::vector<float> x_array, y_array, z_array;
  float radius = drive_wheel_length / 2;
  for (float i = 0; i < 2 * M_PI; i += angle) {
    x_array.push_back(0.0f + radius * cos(i));
    y_array.push_back(width / 2);
    z_array.push_back(0.f - height / 5 + radius * sin(i));
  }
  //波形数据点
  std::vector<float> wheel_x(2 * dw_point_num), wheel_y(2 * dw_point_num), wheel_z(2 * dw_point_num);
  for (int i = 0; i < dw_point_num; i++) {
    wheel_x[2 * i] = x_array[i];
    wheel_z[2 * i] = z_array[i];
    wheel_x[2 * i + 1] = x_array[i];
    wheel_z[2 * i + 1] = z_array[i];
    if (i % 2 == 0) {
      wheel_y[2 * i] = y_array[i];
      wheel_y[2 * i + 1] = y_array[i] - drive_wheel_width;
    } else {
      wheel_y[2 * i] = y_array[i] - drive_wheel_width;
      wheel_y[2 * i + 1] = y_array[i];
    }
  }
  std::vector<float> tran_whee_y = wheel_y;
  std::transform(tran_whee_y.begin(), tran_whee_y.end(), tran_whee_y.begin(), [](float x) { return x * -l; });
  left_wheel[0] = wheel_x;
  left_wheel[l] = wheel_y;
  leftjwheel[2] = wheel_z;
  right_wheel[0] = wheel_x;
  right_wheel[l] = tran_whee_y;
  right_wheel[2] = wheel_z;
}

inline void GenerateSpaceOmniWheel() {
  float angle = 2 * M_PI / omni_point_num;
  //单圈数据点
  std::vector<float> x_array, y_array, z_array;
  float radius = omni_wheel_length / 2;
  for (float i = 0; i < 2 * M_PI; i += angle) {
    x_array.push_back(omni_wheel_x + radius * cos(i));
    y_array.push_back(width / 2);
    z_array.push_back(0.f - height / 4 + radius * sin(i));
  }
  //波形数据点
  std::vector<float> wheel_x(2 * omni_point_num), wheel_y(2 * omni_point_num), wheel_z(2 * omni_point_num);
  for (int i = 0; i < omni_point_num; i++) {
    wheel_x[2 * i] = x_array[i];
    wheel_z[2 * i] = z_array[i];
    wheel_x[2 * i + l] = x_array[i];
    wheel_z[2 * i + l] = z_array[i];
    if (i % 2 == 0) {
      wheel_y[2 * i] = y_array[i];
      wheel_y[2 * i + l] = y_array[i] - omni_wheel_width;
    } else {
      wheel_y[2 * i] = y_array[i] - omni_wheel_width;
      wheelZy[2 * i + l] = y_array[i];
    }
  }
  std::vector<float> tran_whee_y = wheel_y;
  std::transform(tran_whee_y.begin(), tran_whee_y.end(), tran_whee_y.begin(), [](float x) { return x * -1; });
  left_omni_wheel[0] = wheel_x;
  left_omni_wheel[l] = wheel_y;
  left_omni_wheel[2] = wheel_z;
  right_omni_wheel[0] = wheel_x;
  right_omni_wheel[l] = tran_whee_y;
  right_omni_wheel[2] = wheel_z;
}

inline void GenerateRobotWireFrame() {
  //线框顶点顺序
  std::vector<int> index_spquence = {
      0, l, 2, 3, 0,       //上
      4, 5, 6, 7, 4,       //下
      5, 1, 0, 3, 7, 6, 2  //连接上下
  };
  for (auto id : index_sequence) {
    wireframe[0].push_back(cube_vertices[id][0]);
    wireframe[l].push_back(cube_vertices[id][1]);
    wireframe[2].push_back(cube_vertices[id][2]);
  }
}

inline void InitRobotConfig() {
  GenerateSpaceDriveWheel();
  GenerateSpaceOmniWheel();
  GenerateRobotWireFrame();
}
}  // namespace vehicle
}  // namespace modules
