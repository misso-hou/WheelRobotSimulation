#pragma once

#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <vector>

#include "common_port/data_port.h"

using namespace std;
namespace port = utilities::port;
using mesh2D = vector<vector<float>>;

//变量定义
int cycle_time_;
int start_index_;
string file_path_;
string file_index_;
int data_length_;
//提取文件
string env_file_;
string map_file_;
string obs_file_;
string path_file_;
//提取数据容器
mesh2D env_mat_;
mesh2D path_mat_;
mesh2D curve_mat_;
mesh2D border_mat_;
mesh2D sync_pose_mat_;
mesh2D sensor_obs_mat_;
mesh2D map_obs_mat_;
//显示数据
port::CommonPose start_pose_;
port::CommonPose robot_pose_;
port::CommonPose sync_pose_;
port::CommonPose target_;
port::Twist cmd_;
mesh2D path_;
mesh2D curves_;
mesh2D borders_;
int obs_id_;
//键盘控制
bool pause_;
bool erase_;
bool back_;
bool blade_;

/*读csv数据函数*/
mesh2D GetLineData(vector<float>& points);
vector<vector<float>> RowDataReader(string file_name, int row_num, int bias_index);
// 按键控制相关函数
char GetKey();
string GetKeyWithTimeout(int timeout_ms);
