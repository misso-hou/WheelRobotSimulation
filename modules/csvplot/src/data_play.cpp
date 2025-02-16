#include "data_play.h"

#include <sys/select.h>
#include <termios.h>
#include <thread>
#include <unistd.h>

#include "animation.h"
#include "facilities/base_time_struct.h"

using namespace std;
namespace Anim = modules::animation;
Anim::Animation *Animator = Anim::Animation::GetInstance();

const double record_map_resolution = 0.0043;

/**
 *@brief:获取绘制曲线数据
 *@param:
 *     axes:图像axes
 *     robot_pose:实时定位
 */
mesh2D GetLineData(vector<float> &points) {
  vector<float> x_array, y_array;
  if (points.size() % 2 != 0) {
    points.pop_back();
  }
  int points_num = points.size() / 2;
  for (uint i = 0; i < points.size(); i++) {
    i < (uint)points_num ? x_array.push_back(points[i]) : y_array.push_back(points[i]);
  }
  mesh2D line_data(2);
  line_data[0] = x_array;
  line_data[1] = y_array;
  return line_data;
}

/**
 * @brief:按行读取csv数据(分组数据需要设置组中对应的行序列)
 * @param:
 *      file_name:csv文件路径
 *      row_num:csv每组数据的行数(例如三行为一组)
 *      bias_index:数据在csv数据组中对应的行索引
 * @return:csv数据转成的二维数组数据
 */
vector<vector<float>> RowDataReader(string file_name, int row_num, int bias_index) {
  ifstream file(file_name);  // csv文件导入
  if (!file.is_open()) {
    cout << file_name << "----->"
         << "文件不存在" << endl;
  }
  if (file.eof()) {
    cout << file_name << "----->"
         << "文件为空" << endl;
  }
  vector<vector<float>> all_data;
  string line;
  int line_index = 1;
  while (getline(file, line)) {  // 按行提取csv文件
    // 将障碍物数据中index的倍数行提取
    if ((line_index - bias_index) % row_num == 0) {  // note:障碍物三行数据为一组
      stringstream s1(line);
      string charItem;
      float fItem = 0.f;
      vector<float> one_line_data;
      /*提取行点集数据*/
      while (getline(s1, charItem, ',')) {
        fItem = stof(charItem);
        one_line_data.push_back(fItem);
      }
      all_data.push_back(one_line_data);
    } else {
      line_index++;
      continue;
    }
    line_index++;
  }
  // ifs.close();
  return all_data;
}

/**
 * @brief:获取一个字符输入(不需要按enter)
 * @return:键盘输入的字符
 */
char GetKey() {
  struct termios oldt, newt;
  char ch;
  tcgetattr(STDIN_FILENO, &oldt);           // 获取当前终端设置
  newt = oldt;                              // 复制当前设置
  newt.c_lflag &= ~(ICANON | ECHO);         // 禁用回显和规范模式
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);  // 设置新的终端设置
  ch = getchar();                           // 获取一个字符
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);  // 恢复原来的终端设置
  return ch;
}

/**
 * @brief:获取一个字符输入 (不需要按enter),并处理方向键的多字节序列
 * @return:键盘输入的字符
 */
string GetKeyWithTimeout(int timeout_ms) {
  struct termios oldt, newt;
  fd_set readfds;
  struct timeval tv;
  char ch;
  string input = "";

  // 获取当前终端设置
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);         // 禁用回显和规范模式
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);  // 设置新的终端设置

  FD_ZERO(&readfds);               // 清除文件描述符集
  FD_SET(STDIN_FILENO, &readfds);  // 将标准输入文件描述符添加到集

  // 设置超时时间(10毫秒)
  tv.tv_sec = 0;                   // 秒
  tv.tv_usec = timeout_ms * 1000;  // 微秒(10毫秒 = 10000微秒)

  // 使用 select() 检查是否有输入
  int result = select(STDIN_FILENO + 1, &readfds, NULL, NULL, &tv);

  if (result > 0) {  // 如果有输入
    // 获取一个字符
    ch = getchar();
    input += ch;
    // 如果是ESC (27), 可能是方向键的开始字符
    if (ch == 27) {
      ch = getchar();  // 获取下一个字符
      input += ch;
      if (ch == '[') {
        ch = getchar();  // 获取方向键的最后一个字符
        input += ch;
      }
    }
  }
  // 恢复终端设置
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

  return input;
}

/**
 * @brief：数据播放键盘交互控制
 * @param:
 * @return: 键盘输入的字符
 */
bool KeyboardCtrl(int &index) {
  static int copy_time = 0;
  // 暂停&快进&后退控制
  back_ = false;
  if (!pause_) {
    string input = GetKeyWithTimeout(1);  //设置1秒超时
    // 暂停程序
    if (input == " ") {
      cout << "\033[33m 循环暂停，按任意键继续前进一帧，按<-后退一帧\33[0m" << endl;
      pause_ = true;
    }
    // 退出程序(按 'q' 退出循环)
    else if (input == "q") {
      cout << "退出程序!!!" << endl;
      return false;
    }
    // 上键速度X2
    else if (input == "\033[A") {
      cycle_time_ /= 2;
      cycle_time_ = max(1, cycle_time_);
      cout << "loop step cycle_time_ :" << cycle_time_ << "(us)" << endl;
    }
    // 下键速度/2
    else if (input == "\033[B") {
      cycle_time_ *= 2;
      cycle_time_ = min(cycle_time_, 100);
      cout << "loop step cycle_time_ :" << cycle_time_ << "(us)" << endl;
    }
    // 右键快进
    else if (input == "\033[C") {
      index += 300;
      index = min(data_length_ - 1, index);
      erase_ = true;
    }
    // 左键倒退
    else if (input == "\033[D") {
      index -= 100;
      index = max(index, 0);
      erase_ = true;
    }
    copy_time = cycle_time_;
  }
  // 空格暂停
  else {
    cycle_time_ = 5;  // 防止键盘输入等待
    //等待用户输入（持续等待）
    char key = GetKey();
    // 方向按键识别
    if (key == 27) {             // 方向键的起始字符是 ESC (27)
      char nextChar = GetKey();  // 获取方向键的第二个字符
      if (nextChar == '[') {
        nextChar = GetKey();  // 获取最后一个字符
        if (nextChar == 'D') {
          index--;
          index = max(index, 0);
          erase_ = true;
          back_ = true;
        }
      }
    }
    // 退出阻塞模式
    if (key == 'c') {
      cout << "继续循环..." << endl;
      pause_ = false;
      erase_ = false;
      cycle_time_ = copy_time;
    }
  }
  //数据结束后控制逻辑
  if (index == data_length_ - 1) {
    cout << "\033[31m !!!已显示全部数据!!! \33[0m" << endl;
    cout << "\033[32m 按 'c' 继续程序，'q' 退出程序。\33[0m" << endl;
    //等待用户输入
    char key = GetKey();
    if (key == "c") {
      index = 0;
      erase_ = true;
    } else if (key == "q") {
      cout << "!!!退出程序!!!" << endl;
      return false;  //按'q'退出循环
    } else {
      cout << "***再次确认下一步动作***" << endl;
      index--;
    }
  }

  return true;
}

void SetParam(int argc, char *argv[]) {
  //文件路径配置
  char currentPath[FILENAME_MAX];
  getcwd(currentPath, sizeof(currentPath));
  string current_path = string(currentPath);
  // 设置播放速度(默认周期20ms)
  cycle_time_ = argc >= 2 ? stoi(argv[1]) : 10;
  // 播放位置设置（tick累计)
  start_index = argc >= 3 ? atoi(argv[2]) : 0;
  // 指定文件夹序号(每个文件夹例如csv01,csv02...存放多组数据）
  file_path_ = argc >= 4 ? argv[3] : "";
  file_path_ = current_path + "/csvLog" + file_path_ + "/";
  // 指定文件夹内文件序号(用于读取某个文件内部特定索引对应的数据(一次跑机根据按下stop次数会记录多组数据))
  file_index_ = argc >= 5 ? argv[5] : "";
  file_index_ = file_index_ + ".csv";
}

/**
 *@brief:csv数据提取
 */
void ExtractData() {
  //数据文件路径
  env_file_ = file_path_ + "tracking_data" + file_index_;
  obs_file_ = file_path_ + "obs_data" + file_index_;
  path_file_ = file_path_ + "path_classify" + file_index_;
  map_file_ = file_path_ + "static_map.png";
  //数据提取
  env_mat_ = RowDataReader(env_file_, 2, 1);
  data_length_ = env_mat_.size();
  // path
  path_mat_ = RowDataReader(path_file_, 3, 1);
  curve_mat_ = RowDataReader(path_file_, 3, 2);
  border_mat_ = RowDataReader(path_file_, 3, 3);
  // obs
  sync_pose_mat_ = RowDataReader(obs_file_, 3, 1);
  sensor_obs_mat_ = RowDataReader(obs_file_, 3, 2);
  map_obs_mat_ = RowDataReader(obs_file_, 3, 3);
  // 显示数据提取
  start_pose_.x = sync_pose_mat_[0][0];
  start_pose_.y = sync_pose_mat_[0][1];
  start_pose_.theta = sync_pose_mat_[0][2];
}

/**
 *@brief:更新单帧数据
 */
void UpdateData(const int index) {
  //环境元素
  auto env_row = env_mat_[index];
  robot_pose_ = port::CommonPose(env_row[0], env_row[1], env_row[2]);
  target_ = port::CommonPose(env_row[7], env_row[8], 0.f);
  cmd_ = port::Twist(env_row[5], env_row[6]);
  //路径数据
  if ((int)env_row[9] > 0) {
    int path_id = (int)env_row[9] - 1;
    path_ = GetLineData(path_mat_[path_id]);
    curves_ = GetLineData(curve_mat_[path_id]);
    borders_ = GetLineData(border_mat_[path_id]);
  }
  //障碍物数据
  obs_id_ = max(0, (int)env_row[14] - 1);  //障碍物数据时间对齐点
  auto local_pose = sync_pose_mat_[obs_id_];
  sync_pose_ = port::CommonPose(local_pose[0], local_pose[1], local_pose[2]);
  //刀盘显示
  blade_ = cycle_time_ < 4 ? false : true;
}

/*
 * ---------数据回放使用方法----------：
 * 执行命令：./csvPlt+"播放速度设置“+”播放位置设置“+”csv文件夹序号“+”csv文件夹内部文件序号“
 * 播放速度默认为1
 * 播放位置默认从头开始
 * 文件夹默认为csv文件不带后缀序号
 * csv文件内部默认只有一组数据
 */
int main(int argc, char *argv[]) {
  pybind11::scoped_interpreter guard{};
  SetParam(argc, argv);
  ExtractData();
  Animator->InitializePlt(start_pose_, Anim::Mode::LOGGER);
  for (int i = start_index_; i < data_length_;)  //数据行遍历
  {
    //键盘控制
    if (!KeyboardCtrl(i)) break;
    int64_t start_time = TimeToolKit::TimeSpecSysCurrentMs();
    UpdateData(i);
    Animator->SetTrackingData(robot_pose_, cmd_, path_, curves_, borders_, target_, erase_, blade_);
    Animator->SetObsData(sync_pose_, sensor_obs_mat_[obs_id_], map_obs_mat_[obs_id_]);
    /*------动画显示-----*/
    Animator->EnvironmentDisplay();
    Animator->CmdMonitor(600);
    Animator->ObsMonitor();
    Animator->SpaceRobot(1.5f);
    int64_t end_time = TimeToolKit::TimeSpecSysCurrentMs();
    int64_t remaining_T = cycle_time_ - (end_time - start_time);
    if (remaining_T > 0) {
      this_thread::sleep_for(chrono::milliseconds(remaining_T));
    }
    if (!back_) i++;
    erase_ = false;
  }
  return 0;
  pybind11::finalize_interpreter();
}