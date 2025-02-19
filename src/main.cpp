#include <boost/geometry.hpp>
#include <iostream>

#include "animation.h"
#include "control_center.h"
#include "data_center.h"
#include "data_logger.h"
#include "environment.h"
#include "facilities/rate_controller.h"
#include "grid_map.h"
#include "planning.h"
#include "sensor.h"
#include "simulant_robot.h"

using namespace utilities;
using namespace modules;

//变量设置
const float POSE_NOISE = 2.0f;
const float YAW_NOISE = 0.0f;
const port::CommonPose INIT_POSE(103.392, 56.0, -3.14f);
bool path_update_flag_;
bool obs_update_flag_;
port::TrackingInternalState ctrl_state;
std::vector<port::CommonPose> task_path;
vector<Eigen::Vector2f> boundary;

//功能模块
logger::Logger* Data_Logger = logger::Logger::GetInstance();
animation::Animation* Animator = animation::Animation::GetInstance();
datacenter::DataCenter* DC_Instance = datacenter::DataCenter::GetInstance();
control::ControlCenter* Ctrl_Center = control::ControlCenter::GetInstance();
env::Environment Obs_Env;
vehicle::SimulantRobot Virtual_Robot;
sensor::LidarSensor Lidar(2.0f, 11.0f);     // 1度的传感器角度分辨率
planning::PathPlanner Path_Planner(0.04f);  // 0.04的路径距离分辨率
gridmap::GridMap Grid_Map(6.0f, 6.0f, 0.05f, 2.0f);

//数据记录线程
void InfoLoggerThread() {
  utilities::facilities::RateController rt{50};
  while (true) {
    Data_Logger->LoggerLoop();
    rt.Spin();
  }
}

void ObsInfoLoggerThread() {
  utilities::facilities::RateController rt{12};
  while (true) {
    Data_Logger->ObsLoggerLoop();
    rt.Spin();
  }
}

//仿真模块线程
void AnimationThread() {
  pybind11::initialize_interpreter();
  auto start_pose = DC_Instance->GetRobotPose();
  Animator->InitializePlt(start_pose, animation::Mode::SIMULATION);
  utilities::facilities::RateController rt{50};
  while (true) {
    Animator->AnimationLoop();
    rt.Spin();
  }
  pybind11::finalize_interpreter();
}

//传感器数据更新线程
void SensorUpdateThread() {
  utilities::facilities::RateController rt{SIMULATION_RATE * 10};
  while (true) {
    auto env_obs = DC_Instance->GetPlotObsPoints();
    auto robot_pose = DC_Instance->GetRobotPose();
    auto sim_ai_obs = Lidar.UpdateAiSensor(robot_pose, env_obs);
    DC_Instance->SetAiObs(sim_ai_obs);
    auto sim_lidar_data = Lidar.GetLidarData();
    DC_Instance->SetLidarData(sim_lidar_data);
    // HACK:注意位姿与传感器数据不是严格意义上的时间同步，因为分开上传(最好放在一起上传数据)
    DC_Instance->SetAiSyncRobotPose(robot_pose);
    rt.Spin();
  }
}

//地图数据更新线程
void MapUpdateThread() {
  utilities::facilities::RateController rt{SIMULATION_RATE * 10};
  while (true) {
    auto sensor_data = DC_Instance->GetAiObs();
    auto sync_pose = DC_Instance->GetAiSyncRobotPose();
    auto map_obs = Grid_Map.GridMapUpdate(sensor_data, sync_pose);
    DC_Instance->SetMapObs(map_obs);
    rt.Spin();
  }
}

//控制模块线程
void ControlModuleThread() {
  utilities::facilities::RateController rt{SIMULATION_RATE * 50};
  Ctrl_Center->Init();
  while (true) {
    Ctrl_Center->ControlThread();
    rt.Spin();
  }
}

//障碍物更新函数(与主线程一致)
void UpdateEnvironment() {
  utilities::facilities::RateController rt{SIMULATION_RATE * 10};
  while (true) {
    if (obs_update_flag_) {
      auto robot_pose = DC_Instance->GetRobotPose();
      // 生成障碍物
      Obs_Env.InitEnv(8, robot_pose);
      obs_update_flag_ = false;
    } else {
      // 更新障碍物数据
      Obs_Env.EnvUpdate();
    }
    //数据交互
    auto plot_obs_points = Obs_Env.GetShowObs();
    auto obs_points_cloud = Obs_Env.GetObsPoints();
    DC_Instance->SetPlotObsPoints(plot_obs_points);
    DC_Instance->SetObsPointsCloud(obs_points_cloud);
    rt.Spin();
  }
}

//路径规划模块
void PathPlanning(bool& update_flag) {
  if (update_flag) {
    auto start_pose = DC_Instance->GetRobotPose();
    // 曲线路径
    // task_path = Path_Planner.PlanningCurvePath(start_pose,10,0.5f,6.f);
    // 直线路径
    // task_path = Path_Planner.PlanningStraightPath(start_pose,0.6f,20.0f);
    // 生成多目标点(方位控制)
    // task_path = Path_Planner.PlanningMultiTarget(start_pose,10.0f);
    // 椭圆轮廓路径
    // task_path = Path_Planner.PlanningEllipPath(start_pose,6.0,3.0);
    if (task_path.empty()) {
      task_path = Path_Planner.PlanningEllipPath(start_pose, 12, 8);
      boundary = Path_Planner.GetBoundary();
      DC_Instance->SetBoundary(boundary);
    }
    update_flag = false;
  }
}

// 虚拟机器仿真线程(需要最先运行,保证定位数据的可见性)
void RobotSimulation() {
  float frequency = 50.0f;
  utilities::facilities::RateController rt{SIMULATION_RATE * frequency};
  static bool initialize_flag = true;
  port::CommonPose real_time_robot_pose;
  while (true) {
    if (initialize_flag) {
      // 初始化定位
      float T = 1.0f / frequency;
      Virtual_Robot.Init(INIT_POSE, T);
      initialize_flag = false;
      real_time_robot_pose = INIT_POSE;
    } else {
      auto cmd_vel = DC_Instance->GetCmdVel();
      // 更新机器位姿
      real_time_robot_pose = Virtual_Robot.UpdatePose(cmd_vel, POSE_NOISE, YAW_NOISE);
    }
    //数据交互
    DC_Instance->SetRobotPose(real_time_robot_pose);
    rt.Spin();
  }
}

int main() {
  //初始化各个功能模块
  path_update_flag_ = true;
  obs_update_flag_ = true;
  ctrl_state = port::TrackingInternalState::INIT;
  DC_Instance->SetRobotPose(INIT_POSE);
  Data_Logger->CsvRecorderInit();
  // NOTE:首先创建虚拟机器线程,保证定位可见性
  std::thread robot_simulation_thread(RobotSimulation);
  // 创建其他功能线程
  std::thread data_record_thread(InfoLoggerThread);
  std::thread obs_data_record_thread(ObsInfoLoggerThread);
  std::thread animation_thread(AnimationThread);
  std::thread sensor_thread(SensorUpdateThread);
  std::thread control_thread(ControlModuleThread);
  std::thread Environment_thread(UpdateEnvironment);
  std::thread map_thread(MapUpdateThread);
  //主程序线程
  utilities::facilities::RateController rt{SIMULATION_RATE * 50};
  while (true) {
    // std::cout << "Main thread running..." << std::endl;
    /*****路径生成*****/
    PathPlanning(path_update_flag_);
    // 控制任务设置(实时任务)
    // auto ctrk_state = Ctrl_Center->RealTimeTaskTest();
    // 控制功能函数测试
    // ctrl_state = Ctrl_Center->OnceSetTaskTest(task_path);
    // 正常控制任务下发
    if (ctrl_state == port::TrackingInternalState::INIT) {
      auto ctrl_task = port::TrackingTask{};
      ctrl_task.task_type = port::TaskType::PURSUIT;
      ctrl_task.ref_cmd.linear = 1.0f;
      Ctrl_Center->SetControlTask(ctrl_task, task_path, control::function::CtrlALG::DYM_SYS);
    } else if (ctrl_state == port::TrackingInternalState::SUCCESS) {
      cout << "control task finished! new task staring..." << endl;
      ctrl_state = port::TrackingInternalState::INIT;
      path_update_flag_ = true;
      obs_update_flag_ = true;
      continue;
    }
    ctrl_state = Ctrl_Center->GetControlResult();
    rt.Spin();
  }

  // 等待子线程结束
  robot_simulation_thread.join();
  data_record_thread.join();
  obs_data_record_thread.join();
  animation_thread.join();
  sensor_thread.join();
  control_thread.join();
  Environment_thread.join();
  map_thread.join();

  return 0;
}