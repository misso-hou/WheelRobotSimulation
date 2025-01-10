#pragma once
#include <iostream>
#include <cmath>
#include <vector>
#include <string.h>
#include "common_port/data_port.h"

using namespace std;
namespace port = utilities::port;

namespace modules {
namespace control { 
namespace algorithm{

class PathClassify {
    private:
        float friction_factor_;	//摩擦系数，约束车辆过弯最大速度
        float compound_curve_limit_v_ = 0.3; // 复杂曲线的跟踪速度约束
        float length_thd_;	//短/长直线定义阈值
        float angle_thd_;	//曲线拐点定义阈值
        int adjust_index_;	//过短曲线点(方向突变杂点)过滤阈值
        float min_curve_v_;	//最小曲线速宜约束
    public:
        PathClassify(float friction, float length, float angle, int num, float min_v) { 
            friction_factor_ = friction;
            length_thd_ = length;
            angle_thd_ = angle;
            adjust_index_ = num;
            min_curve_v_ = min_v;
        }
        PathClassify() {}
        ~PathClassify(){};
        vector<port::DangerousBorder> DangerousBorderClassify(const vector<port::CommonPose>& smooth_path);
        vector<port::PathCurve> CurveClassify(vector<port::CommonPose>& smooth_path);
        vector<port::PathCurve> ExtractSharpCurves(vector<port::PathCurve>& curves);
        void CalSharpCurveV(vector<port::PathCurve>& curves);
        vector<port::PathCurve> GetSharpCurves(vector<port::CommonPose>& path);
        vector<port::DangerousBorder> GetDangerousBorder(vector<port::CommonPose>& smooth_path);
};

}
}
}
