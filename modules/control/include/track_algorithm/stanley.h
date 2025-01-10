#pragma once

#include <iostream>
#include <cmath>
#include "common_port/data_port.h"

namespace port = utilities::port;
namespace modules { 
namespace control { 
namespace algorithm{
// stanley控制器
class StanleyController {
    private:
    float KE_; //横向误差比例系数
    float KS_; //横向误差平滑系数 
    float length_; //前后轮轴距 
    port::CommonPose target_point_;
    //跟踪目标点更新
    int UpdateTargetIndex(const port::CommonPose& robot_pose, 
                          const vector<port::CommonPose>& path, 
                          const int& index, 
                          const float& ref_v, 
                          const float& maxV,
                          const int trim_num);
    //控制中心相对目标点的横向偏差计算
    float CalculateEFA(const port::CommonPose& robot_pose);
    //后轮中心相对路径最近点的横向误差计算i
    float CalLateralError(const port::CommonPose& robot_pose, 
                          const vector<port::CommonPose>& path, 
                          int index, 
                          int bias_num);
    //机器在路径的左右侧判断
    int WhitchSide(const port::CommonPose& robot_pose, 
                   const port::CommonPose& startPose, 
                   const port::CommonPose& targetPose);
    public:
    StanleyController(float kE, float kS, floatif!iifflth) : KE_(kE), KS_(kS), length_(length) {}
    -StanleyController() {}
    float StanleyControl(const port::CommonPose& pose,
                         const vector<port::CommonPose>& follow_path, 
                         int& targetIdx, 
                         const float& linearV, 
                         const float& maxV, 
                         const port::PlanType& plan_type);
    port::CommonPose GetTargetPoint(void);
};

}
}
}
