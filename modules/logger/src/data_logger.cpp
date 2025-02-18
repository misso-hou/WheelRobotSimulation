#include "data_logger.h"

#include "data_center.h"
#include "facilities/base_time_struct.h"

using namespace modules::datacenter;
using namespace utilities::port;

namespace modules {
namespace logger {

DataCenter* DC_Instance = DataCenter::GetInstance();

/*
 *@breif:初始化函数->创建清空旧文件夹，新建csv文件夹，创建csv记录文件
 */
void Logger::CsvRecorderInit() {
  //更新eSV文件夹
  std::string command = "rm -r " + dir;
  system(command.c_str());      //删除原有文件夹
  mkdir(dir.c_str(), S_IRWXU);  //新建文件夹
  CreateCSVfile();
  //初始化变量
  path_recorder_count_ = 0.0f;    //路径记录tick(匹配跟踪路径与跟踪数据)
  last_recorder_time_stamp_ = 0;  //数据记录时间戳(大致统计机器运行时间)
  duration_time_ = 0.0f;
  obs_update_tick_ = 0;  //解析数据保证障碍物数据行索引与tick同步
}

/*
 * @breif:约束CSV文件记录长度
 */
void Logger::CoverCsvSizeLimit(uint& count, int index) {
  // csv转存特定长度的数据
  if (count > CSV_Length_Limit) {
    count = 0;
    time_tick_count_ = count;
    string file_index = to_string(index) + "_0";
    CoverCopyCSVfile(file_index);
  }
  // csv数据未超过特定长度,更新时间轴数据
  else {
    time_tick_count_ = count;
    count++;
  }
}

/*
 * @breif: 创建全覆盖对应的csv文件，清空对应的文件
 */
void Logger::CreateCSVfile() {
  csv_writer.CSVcreate(tracking_file + file_type);  // 记录跟踪误差
  csv_writer.CSVcreate(obs_data_file + file_type);
  csv_writer.CSVcreate(time_record_file + file_type);
  csv_writer.CSVcreate(path_file + file_type);
}

/*
 * @brief: 转存csv记录数据(跟踪数据，障碍物数据，时间记录数据，路径数据不转存)
 * @param:
 *    index:文件序号(按一次stop,转存一次csv数据，文件转存叠加)
 */
void Logger::CoverCopyCSVfile(string subscript) {
  //转存CSV数据
  string file_index = subscript;
  rename((tracking_file + file_type).c_str(), (tracking_file + file_index + file_type).c_str());
  rename((obs_data_file + file_type).c_str(), (obs_data_file + file_index + file_type).c_str());
  rename((time_record_file + file_type).c_str(), (time_record_file + file_index + file_type).c_str());
  rename((path_file + file_type).c_str(), (path_file + file_index + file_type).c_str());
  //新建csv文件，重新开始记录csv数据
  CreateCSVfile();
}

/*
 * @brief:记录跟踪数据
 */
void Logger::TrackingDataRecord() {
  /*根据不同工作场景写入各自文件*/
  string tracking_file_dir;
  tracking_file_dir = tracking_file + file_type;
  int64_t current_time_stamp = TimeToolKit::TimeSpecSysCurrentMs();  // 获取当前时间戳
  if (last_recorder_time_stamp_ == 0) {
    duration_time_ = 0.0f;
  } else {
    duration_time_ += ((current_time_stamp - last_recorder_time_stamp_) / 1000.0f);
  }
  {
    std::lock_guard<std::mutex> lk(obs_data_mutex_);
    copy_obs_updata_tick = obs_update_tick_;
  }
  vector<float> data = {robot_pose_.x,
                        robot_pose_.y,
                        robot_pose_.theta,  //记录轨迹
                        duration_time_,
                        error_,  //记录时间戳(主线程频率50HZ)和误差
                        linear_speed_,
                        angular_speed_,  //记录速度
                        target_pose_.x,  //记录跟踪目标点
                        target_pose_.y,
                        path_recorder_count_,
                        observe_v_,  //反馈速度记录
                        observe_w_,
                        pitch_,  //姿态角
                        roll_,
                        copy_obs_updata_tick};
  /*CSV记录跟踪数据*/
  csv_writer.RecordWithTime(tracking_file_dir, data);
  last_recorder_time_stamp_ = current_time_stamp;
}

/*
 * @brief: 记录膨胀障碍物&原始障碍物&dwa绕障数据
 * @params:
 *	  c_or_u:上下车模式选择
 *	  obs_point_data:膨胀障碍物点云信息
 *	  ai_obs_data:原始障碍物点云信息
 *    pose_2D_array:dwa预测路径信息
 */
void Logger::ObsDataRecord() {
  std::lock_guard<std::mutex> lk(obs_data_mutex_);
  string obs_file;
  obs_file = obs_data_file + file_type;

  ofstream obs_recorder;
  obs_recorder.open(obs_file, ios::app);  // 以追加形式打开csv文件
  /*写入时间同步定位*/
  obs_recorder << time_sync_pose_.x << "," << time_sync_pose_.y << "," << time_sync_pose_.theta << endl;
  /*写入传感器障碍物数据*/
  if (sensor_obs_.empty()) {
    obs_recorder << 0.0 << "," << 0.0 << endl;
  } else {
    for (auto point : sensor_obs_) {
      obs_recorder << fixed << setprecision(3) << point.x << ",";
    }
    for (auto point : sensor_obs_) {
      obs_recorder << fixed << setprecision(3) << point.y << ",";
    }
    obs_recorder << endl;  // 换行
  }
  /*写入地图障碍物数据*/
  if (grid_map_obs_.empty()) {
    obs_recorder << 0.0 << "," << 0.0 << endl;
  } else {
    for (auto point : grid_map_obs_) {
      obs_recorder << fixed << setprecision(3) << point.x << ",";
    }
    for (auto point : grid_map_obs_) {
      obs_recorder << fixed << setprecision(3) << point.y << ",";
    }
    obs_recorder << endl;  // 换行
  }
  obs_recorder.close();
  //更新障碍物更新标记，方便解析时间同步
  obs_update_tick_++;
}

void Logger::SpecialPathRecord(const poses& origin_border_path, const poses& sharp_curves, const poses& dangurous_border) {
  ofstream local_csv_writer;
  //原始边界记录
  local_csv_writer.open(path_file + file_type, ios::app);
  for (auto p : origin_border_path) {
    local_csv_writer << p.x << ",";
  }
  for (auto p : origin_border_path) {
    local_csv_writer << p.y << ",";
  }
  local_csv_writer << endl;

  //急转弯记录
  if (sharp_curves.empty()) {
    local_csv_writer << 0.0 << "," << 0.0 << endl;
  } else {
    for (auto p : sharp_curves) {
      local_csv_writer << p.x << ",";
    }
    for (auto p : sharp_curves) {
      local_csv_writer << p.y << ",";
    }
    local_csv_writer << endl;
  }

  //危险边界记录
  if (dangurous_border.empty()) {
    local_csv_writer << 0.0 << "," << 0.0 << endl;
  } else {
    for (auto p : dangurous_border) {
      local_csv_writer << p.x << ",";
    }
    for (auto p : dangurous_border) {
      local_csv_writer << p.y << ",";
    }
    local_csv_writer << endl;
  }
  local_csv_writer.close();
}

void Logger::GetTrackingRecordData() {
  csv_recorder_switch_ = false;
  //记录器开关
  csv_recorder_switch_ = DC_Instance->GetCsvSwitch();
  //定位
  robot_pose_ = DC_Instance->GetRobotPose();
  //姿态角数据
  static float test_tick = 0.0f;
  test_tick = (test_tick + 0.01f) > 2 * 3.14f ? 0.0f : test_tick + 0.01f;
  pitch_ = 0.26f * sin(test_tick * 0.5f);
  roll_ = 0.26f * sin(test_tick * 0.8f);
  //跟踪路径
  bool new_path_flag = DC_Instance->GetPathUpdateFlag();
  //判断是否有更新的路径
  if (new_path_flag) {
    smooth_path_ = DC_Instance->GetPlanningPath();
    sharp_curves_ = DC_Instance->GetSpCurves();
    sharp_curves_ = DC_Instance->GetDgBorders();
    DC_Instance->SetPathUpdateFlag(false);
    if (!smooth_path_.empty()) {
      path_recorder_count_++;
      SpecialPathRecord(smooth_path_, sharp_curves_, dg_borders_);
    }
  }
  //控制速度
  auto cmd_vel = DC_Instance->GetCmdVel();
  linear_speed_ = cmd_vel.linear;
  angular_speed_ = cmd_vel.angular;
  //反馈逮度
  // observe_v_ = data.linear();
  // observe_w_ = data.angular();
  //控制目标点获取
  target_pose_ = DC_Instance->GetTargetPose();
  // todo：跟踪误差
  error_ = 0.0f;
}

void Logger::GetObsData() {
  // ai障碍物数据获取
  sensor_obs_.clear();
  grid_map_obs_.clear();
  time_sync_pose_ = DC_Instance->GetAiSyncRobotPose();
  //传感器障碍物点数据
  auto ai_obs_points = DC_Instance->GetAiObs();
  for (auto point : ai_obs_points) {
    //机器人坐标系
    sensor_obs_.push_back(ObsPoint(point(0), point(1)));
  }
  //删格地图障碍物数据
  auto map_obs_points = DC_Instance->GetMapObs();
  for (auto point : map_obs_points) {
    //机器人坐标系
    grid_map_obs_.push_back(ObsPoint(point(0), point(1)));
  }
}

//高速数据记录线程函数
void Logger::LoggerLoop() {
  GetTrackingRecordData();
  if (csv_recorder_switch_) {
    TrackingDataRecord();
    // TimeConsumingRecord();
  }
}

//障碍物数据记录线程函数
void Logger::ObsLoggerLoop() {
  GetObsData();
  if (csv_recorder_switch_) {
    ObsDataRecord();
  }
}

}  // namespace logger
}  // namespace modules
