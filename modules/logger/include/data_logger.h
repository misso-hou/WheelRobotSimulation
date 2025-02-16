#pragma once
#include <deque>
#include <iostream>
#include <mutex>
#include <sys/stat.h>
#include <thread>
#include <vector>

#include "common_port/data_port.h"
#include "csv_writer.h"
#include "facilities/rate_controller.h"
#include "facilities/singleton.h"

using namespace std;
namespace port = utilities::port;

namespace modules {
namespace logger {

using poses = vector<port::CommonPose>;

/*csv文件路径*/
const string dir = string{CSV_SOURCE_DIR} + "/csvLog";
const string tracking_file = dir + "/tracking_data";
const string obs_data_file = dir + "/obs_data";
const string path_file = dir + "/path_classify";
const string time_record_file = dir + "/time_consume";
const string file_type = ".csv";

/*csv记录文件参数设置*/
const int CSV_Length_Limit = 50000;  // csv文件记录长度约束

class Logger : public utilities::Singleton<Logger> {
  friend class Singleton<Logger>;

 private:
  Logger(){};
  ~Logger(){};

 public:
  void CoverCopyCSVfile(string subscript);
  void CsvRecorderInit();
  void CoverCsvSizeLimit(uint& count, int index);
  void SpecialPathRecord(const poses& origin_border_path, const poses& dangurous_border, const poses& sharp_curves);
  void LoggerLoop();
  void ObsLoggerLoop();

 private:
  void CreateCSVfile();
  void TrackingDataRecord();
  void ObsDataRecord();
  void TimeConsumingRecord();
  void GetTrackingRecordData();
  void GetObsData();

 private:
  /*跟踪数据*/
  int time_tick_count_ = 0;
  poses smooth_path_;   // 跟踪路径
  poses sharp_curves_;  // 急转弯
  poses dg_borders_;    // 危险边界
  // uint path_index_;	// 路径数据索引
  port::CommonPose robot_pose_;   // 定位
  float linear_speed_;            //	线速度
  float angular_speed_;           //	角速度
  float observe_v_;               //	实际线速度
  float observe_w_;               //	实际角速度
  float track_mode_;              //	控制模式
  float map_command_;            //地图下发控制模式
  port::CommonPose target_pose_;  // 目标点
  float error_;                   // 跟踪误差
  float pitch_;
  float roll_;
  /*障碍物数据*/
  port::CommonPose time_sync_pose_;                // ai时间同步位姿
  vector<vector<port::ObsPoint>> obs_point_data_;  // 瞬时障碍物数据
  vector<port::ObsPoint> sensor_obs_;              // ai障碍函或据
  vector<port::ObsPoint> grid_map_obs_;            // ai障碍物数据
  vector<poses> pose_2D_array_;                    // dwa避障算法预测位姿记录
  int obs_update_tick_;                            // 障碍物更新标记
  int copy_obs_updata_tick;
  /*耗时数据*/
  float loop_time_;  //主逻辑运行周期时间
  float ctrl_time_;  //控制模块耗时
  float map_time_;   //地图取障耗时
  int num_obs_;      //膨胀障碍点云个数
  int num_ai_;       // ai原始障碍点云个数
  /*数据线程读写锁*/
  std::mutex obs_data_mutex_;

 private:
  std::thread data_record_thread_;
  bool csv_recorder_switch_;
  CSVWandR csv_writer;

 private:                             //需要初始化的变量
  float path_recorder_count_;         //路径记录tick（匹配跟踪路径与跟踪数据）
  int64_t last_recorder_time_stamp_;  //数据记录时间戳（大致统计机器运行时间）
  float duration_time_;               //运行时间累计
};
}  // namespace logger
}  // namespace modules
