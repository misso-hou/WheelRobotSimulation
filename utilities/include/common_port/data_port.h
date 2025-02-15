#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>

using namespace std;

namespace utilities {
namespace port {

using vec2f = Eigen::Vector2f;
using mat2f = Eigen::Matrix2f;

struct CommonPose {
  CommonPose() : x{0}, y{0}, theta{0} {}
  CommonPose(float a, float b, float c) {
    x = a;
    y = b;
    theta = c;
    pitch = 0.f;
    roll = 0.f;
    type = 1;  //非危险边界点
  }
  CommonPose(float a, float b, float c, float d, float e) {
    x = a;
    y = b;
    theta = c;
    pitch = d;
    roll = e;
    type = 1;  //非危险边界点
  }
  float x;
  float y;
  float theta;
  float pitch;
  float roll;
  int type;  //判断是否为危险边界
};

//控制任务类型
typedef enum {
  NONE = 0,
  PURSUIT,          //跟踪
  AVOID,            //局部避障
  MOVE_ARC,         //走弧线
  MOVE_LINE,        //走直线
  MOVE_BY_DIS,      //直线行走特定距离
  MOVE_BY_TIME,     //按时间阈值执行
  ROTATE,           //旋转
  ROTATE_BY_ANGLE,  //旋转特定角度
  RANDOM,           //直接下发速度模式
  STOP              //停车
} TaskType;

// 期望速度
struct RefCmd {
  RefCmd() : linear{0}, angular{0}, acceleration{0}, deceleration{0}, wheel_torque{0} {}
  float linear;        // 线速度
  float angular;       // 角速度
  float acceleration;  // 线加速度设置
  float deceleration;  // 线减速度设置
  float wheel_torque;  // 驱动轮电机扭矩
};

// 控制任务阈值
struct TaskThd {
  TaskThd() : angle{0.0f}, distance{0.0f}, radius{0.0f}, time_ms{0} {}
  float angle;     //	旋转角度
  float distance;  //	行走距离
  float radius;    //	划弧半径
  int time_ms;     //	执行前间
};

//规划任务场景
typedef enum {
  COVERCUT,  //	弓字
  P2P,       //	p2p
  SIDE       // 沿边
} PlanType;

// 控制模块任务接口
struct TrackingTask {
  TaskType task_type;  // 任务类型
  RefCmd ref_cmd;      // 期望速度
  TaskThd task_thd;    // 任务执行阈值
  PlanType plan_type;  // 规划类型
};

/*控制速度指令*/
struct Twist {
  Twist() : linear{0}, angular{0}, left_accel{1000}, left_decel{1000}, right_accel{1000}, right_decel{1000} {}
  Twist(float a, float b) {
    linear = a;
    angular = b;
  }
  float linear;
  float angular;
  float left_accel;
  float left_decel;
  float right_accel;
  float right_decel;
};

/*速度控制模式（优先级逐渐提高)*/
typedef enum { NORMAL, SLOWDOWN, SLOWUP_LEVEL1, SLOWUP_LEVEL2, SLOWDOWN_LEVEL1, SLOWDOWN_LEVEL2, SHUTDWON } ActionState;

/*车辆控制物理参数*/
struct ControlParam {
  float tick_time = 0.f;            //控制执行周期
  float min_arround_v = 0.f;        //最小绕障速度
  float min_tracking_v = 0.f;       //最小路径跟踪速度（大弧度降速）
  float angular_acc = 0.f;          //角加速度
  float min_omega = 0.f;            //最小角速度
  float max_omega = 0.f;            //最大角速度
  float heading_error_lim_v = 0.f;  //直线跟踪误差较大时约束线速度
  float acceleration = 0.f;         //加速度约束
  float deceleration = 0.f;         //减速度约束（正常减速，保证平滑，停障减速可以设置更大）
  float dg_limit_v = 0.f;           //危险边界约束速度
};

/*跟踪模块阈值设定*/
struct CtrlThreshold {
  float lateral_error_thd = 0.f;     // 横向偏差阈值
  float heading_error_thd = 0.f;     // 航向偏差阈值
  float smooth_dis_thd = 0.f;        //到达终点速度缓冲距离
  float angle_align_thd = 0.f;       //原地旋转,航向对正角度阈值
  float arrival_accuracy_thd = 0.f;  // 到点精度设置
  int out_curve_dis_thd_ = 0;        //出弯道距离约束阈值（出弯道不立刻加速）
  int short_path_thd_ = 0;           //短路径判断阈值（根据路径点个数）
  int overtime_thd_ = 0;             //终点跟踪超时阈值（任务异常判断）
};

/*控制模块内部状态结果*/
typedef enum { INIT = 0, TRACKING, SUCCESS } TrackingInternalState;

typedef enum { straight = 0, curve } PathType;

/*包含路径类型的跟踪路径接口*/
struct CompoundPath {
  vector<CommonPose> path;
  PathType path_type;
  float ref_linear;
};

typedef enum { simple_curve = 0, compound_curve } CurveType;

struct DangerousBorder {
  std::vector<int> origin_index;  // 危险边界段对应的原始路径点索引
  CommonPose start_point;         // 危险边界段起点
};

struct PathCurve {
  vector<CommonPose> curve;       //曲线路段
  std::vector<int> origin_index;  //曲线路径对应的原始路径点索引
  float curvature;                //曲线去率
  float theta;                    //曲线弧心角
  float limit_v;                  //曲线约束速度
  CurveType type;                 //曲线类型
};

struct ObsPoint {
  ObsPoint(){};
  ObsPoint(float set_x, float set_y) {
    x = set_x;
    y = set_y;
  }
  float x;
  float y;
};

struct SimAiObs {
  std::vector<cv::Point2f> obs_points;
  // type
};

struct Discriminant {
  float A;
  float B;
  float C;
};

// planning trajectory对应点(planning path + speed profile)
struct TrajPoint {
  TrajPoint(){};
  TrajPoint(const CommonPose& set_pose, const float& set_ref_v) {
    pose = set_pose;
    ref_v = set_ref_v;
    K = 1;
    T = 0;
  }
  CommonPose pose;
  float ref_v;  //点期望速度;
  float K;      //点对应曲率
  float T;      //目标时间(用于MPC规划)
};

//动态系统避障算法数据接口
typedef struct {
  Eigen::Vector2f pose;  // point pose
  int clusterID;         // clustered ID
  uint index;            // 原点集中的索引
} ClusterPoint;

/*********算法数据类型定义***********/  // note:display tool与trackingsdk存在交叉编译问题
                                        //角度分区采样点
struct SamplePoint {
  vec2f pose;       //位置
  float polar_dis;  //极作弊半径
  int phase;        //相限
  float angle;      //相限角
};

//虚拟障碍物（障碍物几何数据）
struct VirtualObs {
  vector<vec2f> points;
  vector<vec2f> sides;  // polygon凸边
  vec2f center;
  float min_ratio_dis;  // agents对应最近距离（所有agents）
};

// 绕障方向
typedef enum { left = 0, right, random } AvoidDir;

// 计算障碍物
struct CalculatedObs {
  // vec2f relative_velocity;	//相对机器速度 //note:暂时未用
  // 障碍物计算数据
  vec2f reference_point;  //障碍物特征点（几何中心）
  vec2f nearest_point;    //相对机器最近点
  float nearest_dis;      //相对机器最近距离
  float gamma_dis;        //比例距离系数
  float weight;           //个体权重
  // 向量元素
  vec2f normal_vector;     // 最近点法向量
  vec2f tangent_vector;    // 最近点切向量
  vec2f reference_vector;  // 相对向量
  //势场矩阵元素
  vec2f trim_vector;        //微调
  mat2f basic_matrix;       //特征向量矩阵
  mat2f streching_matrix;   //特征值矩阵
  mat2f orthogonal_matrix;  //正交矩阵
  mat2f modulate_matrix;    //调制矩阵
  vec2f modulate_vector;    //障碍物对应的调制速度方向
  // 绕障方向
  AvoidDir avoid_dir;
};

// 机器参数定义
struct BoundaryCircle {
  float radius;    // 外形参数
  float center_x;  // 圆心局部坐标
  float center_y;
  float safe_margin;  // 安全膨胀距离
  vec2f local_center;
  BoundaryCircle() {}
  BoundaryCircle(float a, int b, float c, float d, float e, float f, float g) {
    radius = a;
    center_x = d;
    center_y = e;
    safe_margin = g;
  }
};

// 障碍物点采样圆
struct SampleCircle {
  int sub_num;     // 采样区间数
  float range;     // 避障范围
  float center_x;  // 圆心局部坐标
  float center_y;
  float radius;  // 更新目标点范围
  vec2f local_center;
  vec2f global_center;
  vec2f heading_dir;
  mat2f jacobian;  // 相对控制中心的亚克比矩阵
};

// 机器轮廓对应的计算元素
struct CircleAgent {
  //机器参数
  BoundaryCircle circle;
  vec2f center_pose;
  // 辅助计算元素
  mat2f jacobian;  // 相对控制中心的亚克比矩阵
  // 障碍物元素
  vector<CalculatedObs> calculated_obstacles;
  float safe_dis;
  // 速度参数
  vec2f cur_vel_vec;   // 当前圆心速度矢量
  vec2f compound_vel;  // 矢量合成modulate速度
  vec2f ref_vel_vec;   // 跟踪期望速度对应的圆心速度矢量
  // todo:trash param
  // debug向量信息
  vec2f cur_v_normal;
  vec2f cur_v_tangent;
  vec2f debug01_vec;
  vec2f debug02_vec;
  vec2f debug03_vec;
  vec2f debug04_vec;
};

}  // namespace port
}  // namespace utilities
