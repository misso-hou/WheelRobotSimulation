#include "control_center.h"

#include "control_base.h"

namespace modules {
namespace control {

namespace func = modules::control::function;
namespace base = modules::control::base;

void ControlCenter::Init() {
  // NOTE:根据机型设置控制参数&控制阈值
  // 控制参数设置
  base::k_cp_.tick_time = 0.02;            // 控制执行周期
  base::k_cp_.min_arround_v = 0.4;         // 最小绕障速度（防止障碍物在车体侧面导致停车）
  base::k_cp_.min_tracking_v = 0.2;        // 最小路径跟踪速度（大弧度降速）
  base::k_cp_.angular_acc = 2.0;           // 角加速度
  base::k_cp_.min_omega = -1.2;            // 最小角速度
  base::k_cp_.max_omega = 1.2f;            // 最大角速度
  base::k_cp_.heading_error_lim_v = 0.1f;  // 直线跟踪误差较大时约束线速度
  base::k_cp_.acceleration = 0.8f;         // 加速度约束
  base::k_cp_.deceleration = -1.0f;        // 减速度约束（正常减速，保证平滑，停障减速可以设置更大）
  base::k_cp_.dg_limit_v = 0.3;            // 危险边界约束速度
  //控制阈值设置
  base::k_ct_.lateral_error_thd = 0.1;     // 横向偏差阈值
  base::k_ct_.heading_error_thd = 8.0f;    // 航向偏差阈值
  base::k_ct_.smooth_dis_thd = 0.2f;       // 到达终点速度缓冲距离
  base::k_ct_.angle_align_thd = 5.0;       // 原地旋转,航向对正角度阈值
  base::k_ct_.arrival_accuracy_thd = 0.1;  // 到点精度设置
  base::k_ct_.out_curve_dis_thd_ = 25;     // 出弯道不加速距离阈值(路径点数)
  base::k_ct_.short_path_thd_ = 10;        // 短路径判断(少于设定阈值，路径不分类)
  base::k_ct_.overtime_thd_ = 10;          // 终点跟踪超时阈值设置(s)
  //功能模块初始化
  base::VP_Instance->Init(base::k_cp_, base::k_ct_);
  tracker_ptr_ = make_shared<func::PathTracker>();
}

port::TrackingInternalState ControlCenter::GetControlResult() {
  std::lock_guard<std::mutex> lk(thread_lock_);
  return ctrl_result_;
}

/**
 * @brief：控制任务初始化
 */
void ControlCenter::TaskReset() {
  base::ResetBasicCtrl(interface_task_);
  if (interface_task_.task_type == port::TaskType::PURSUIT) {
    tracker_ptr_->ResetPathTracker(interface_path_, interface_method_);
  } else {
    // TODO:基础动作库
  }
}

void ControlCenter::SetControlTask(const port::TrackingTask& task, vector<port::CommonPose> path, func::CtrlALG method) {
  std::lock_guard<std::mutex> lk(thread_lock_);
  new_task_ = true;  //保证线程数据安全
  interface_task_ = task;
  interface_path_ = path;
  interface_method_ = method;
}

/**
 *@brief:设置控制任务
 */
port::TrackingInternalState ControlCenter::TaskAllocation() {
  base::k_csv_switch_ = true;
  port::TrackingInternalState task_state;
  //控制任务状态机
  switch (base::k_task_.task_type) {
    case port::TaskType::PURSUIT: {
      task_state = tracker_ptr_->PathTracking();
      break;
    }
    case port::TaskType::MOVE_ARC: {
      break;
    }
    case port::TaskType::MOVE_BY_DIS: {
      break;
    }
    case port::TaskType::MOVE_BY_TIME: {
      break;
    }
    case port::TaskType::ROTATE_BY_ANGLE: {
      break;
    }
    case port::TaskType::MOVE_LINE: {
      break;
    }
    case port::TaskType::ROTATE: {
      break;
    }
    case port::TaskType::RANDOM: {
      break;
    }
    case port::TaskType::STOP: {
      base::Stop();
      break;
    }
    default:
      task_state = port::TrackingInternalState::INIT;
      base::k_cmd_ = port::Twist{};
      base::k_csv_switch_ = false;  // 无控制任务->停止数据记录
      break;
  }
  return task_state;
}

void ControlCenter::SetControlResult(const port::TrackingInternalState& state) {
  std::lock_guard<std::mutex> lk(thread_lock_);
  if (!new_task_) {
    ctrl_result_ = state;
  }
}

/**
 * @brief:控制模块独立线程函数
 */
int32_t ControlCenter::ControlThread() {
  //新任务设定保护
  {
    std::lock_guard<std::mutex> lk(thread_lock_);
    if (new_task_) {
      new_task_ = false;
      TaskReset();
      ctrl_result_ = port::TrackingInternalState::TRACKING;
    }
  }
  //基础模块变量设置
  base::SetRealTimeInfo();
  //根据任务类型调用功能模块
  auto task_state = TaskAllocation();
  // note:线程接口数据保护
  SetControlResult(task_state);
  //总线数据更新
  base::UpdateData();
  return 0;
}

}  // namespace control
}  // namespace modules
