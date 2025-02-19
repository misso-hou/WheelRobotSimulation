#pragma once

#include <iostream>

#include "facilities/singleton.h"
#include "function/path_tracker.h"

namespace modules {
namespace control {

using namespace std;
namespace port = utilities::port;
namespace func = modules::control::function;

class ControlCenter : public utilities::Singleton<ControlCenter> {
  friend class Singleton<ControlCenter>;

 private:
  ControlCenter(){};
  ~ControlCenter(){};

 private:
  port::TrackingInternalState TaskAllocation();
  void SetControlResult(const port::TrackingInternalState& state);

 public:
  void Init();
  port::TrackingInternalState GetControlResult();
  void TaskReset();
  void SetControlTask(const port::TrackingTask& task, vector<port::CommonPose> path = vector<port::CommonPose>{},
                      func::CtrlALG method = func::CtrlALG::PP);
  int32_t ControlThread();

 private:
  shared_ptr<func::PathTracker> tracker_ptr_;

  //接口变量
 private:
  vector<port::CommonPose> interface_path_;
  port::TrackingTask interface_task_;
  func::CtrlALG interface_method_;
  port::TrackingInternalState ctrl_result_;

 private:
  std::mutex thread_lock_;
  bool new_task_;
};
}  // namespace control
}  // namespace modules