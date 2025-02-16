#include "algorithm/velocity_plan/path_classify.h"

#include "control_base.h"
#include "tools/math_tools.h"

namespace mathTools = utilities::mathTools;

namespace modules {
namespace control {
namespace algorithm {

/**
 *@brief:延边复杂路径分类（危险边界）//todo:路径点类型
 *@param:
 *   smooth_path:原始跟踪路径
 *   show_dg_border:按地址返回需要显示的危险边界点集
 * @return:
 *   danger_borders:危险边界路段集合（对应的原始路径点所以， 危险边界路径起点）
 */
vector<port::DangerousBorder> PathClassify::DangerousBorderClassify(const vector<port::CommonPose>& smooth_path) {
  // vector<int> index_array; // 危险边界段对应的原始路径点索引
  port::DangerousBorder temp_border;  // 危险边界段
  vector<port::DangerousBorder> danger_borders;
  // vector<int> show_temp_index;
  for (uint index = 0; index < smooth_path.size(); index++) {
    if (smooth_path[index].type != 1) {  // 安全边界点，继续向后索引危险边界
      /*当前路径点前有未归类的危险边界*/
      if (!temp_border.origin_index.empty()) {
        // show_temp_index.insert(show_temp_index.end(), temp_border.origin_index.begin(), temp_border.origin_index.end());
        danger_borders.push_back(temp_border);
        memset(&temp_border, 0, sizeof(temp_border));
      }
      continue;
    } else {
      if (index == 0) {  //起点为危险边界
        temp_border.origin_index.push_back(index);
        temp_border.start_point = smooth_path[index];
      } else {
        if (!temp_border.origin_index.empty()) {  //在危险边界中，继续记录危险边界点
          temp_border.origin_index.push_back(index);
        } else {
          temp_border.origin_index.push_back(index);
          temp_border.start_point = smooth_path[index];
        }
      }
    }
  }
  /*将遍历剩余未归类的危险边界进行归类*/
  if (!temp_border.origin_index.empty()) {
    danger_borders.push_back(temp_border);
    // show_temp_index.insert(show_temp_index.end(), temp_border.origin_index.begin(), temp_border.origin_index.end())；
  }
  /*提取需要显示的危险边界路段*/
  // for (uint i = 0; i < show_temp_index.size(); i++) {
  //  show_dg_border.push_back(smooth_path[show_temp_index.at(i)]);
  // }
  return danger_borders;
}

/*
 *@brief:跟踪路径曲线分类（简单曲线，复杂曲线（简单曲线+短直线,直线）
 *@param:
 *   smooth_path:原始跟踪路径
 * @return:
 *   curves:分类出来的曲线
 */
vector<port::PathCurve> PathClassify::CurveClassify(const vector<port::CommonPose>& smooth_path) {
  float delta_y2, delta_x2, delta_y1, delta_x1;
  float theta_1, theta_2, delta_theta;
  bool line_flag = true;
  port::PathCurve temp_curve, temp_line, temp_compound_curve;  //临时分类曲线容器
  vector<port::PathCurve> curves;                              //曲线容器，存储路径上的曲线
  float line_length;                                           //临时直线长度
  static bool last_line_flag = false;
  port::CommonPose line_end, line_start;
  /*全路径遍历分类*/
  for (uint i = 0; i < smooth_path.size() - 2; i++) {  //剩余最后三个点不进行分类，默认是直线(有终点减速逻辑)
    /*计算相邻点的角度变化*/
    delta_y1 = smooth_path[i + 1].y - smooth_path[i].y;
    delta_x1 = smooth_path[i + 1].x - smooth_path[i].x;
    delta_y2 = smooth_path[i + 2].y - smooth_path[i + 1].y;
    delta_x2 = smooth_path[i + 2].x - smooth_path[i + 1].x;
    theta_1 = atanf(delta_y1 / delta_x1);
    theta_2 = atanf(delta_y2 / delta_x2);
    delta_theta = mathTools::NormalizeAngle(theta_2 - theta_1) * 180 / M_PI;  //相邻路径点方向偏差(角度)
    /*判断直线还是曲线*/
    if (fabs(delta_theta) > angle_thd_) {
      line_flag = false;
    } else {
      line_flag = true;
    }
    /*情况一：曲线转直线*/
    if (line_flag && last_line_flag != line_flag && !temp_curve.curve.empty()) {
      //复杂曲线未结束，曲线容器插入新的复杂曲线
      if (!temp_compound_curve.curve.empty()) {
        temp_compound_curve.curve.insert(temp_compound_curve.curve.end(), temp_curve.curve.begin(), temp_curve.curve.end());
        temp_compound_curve.origin_index.insert(temp_compound_curve.origin_index.end(), temp_curve.origin_index.begin(),
                                                temp_curve.origin_index.end());
      }
      //曲线前无复杂曲线,根据曲线长短进行归类
      else {
        //短曲线,插入到新直线前段
        if ((int)temp_curve.curve.size() < adjust_index_) {
          temp_line = temp_curve;  //此时新直线还未罗列点，直接将短曲线设置为直线类型
        }
        //长曲线,长曲线归档
        else {
          temp_curve.type = port::CurveType::simple_curve;
          curves.push_back(temp_curve);
        }
      }
      memset(&temp_curve, 0, sizeof(temp_curve));
    }
    /*情况二：直线转曲线*/
    if (!line_flag && last_line_flag != line_flag && !temp_line.curve.empty()) {
      /*计算直线长度*/
      line_end = temp_line.curve.back();
      line_start = temp_line.curve.front();
      line_length = sqrtf(powf(line_end.y - line_start.y, 2) + powf(line_end.x - line_start.x, 2));
      //短直线,直线归入到复杂曲线
      if (line_length < length_thd_) {
        temp_compound_curve.curve.insert(temp_compound_curve.curve.end(), temp_line.curve.begin(), temp_line.curve.end());
        temp_compound_curve.origin_index.insert(temp_compound_curve.origin_index.end(), temp_line.origin_index.begin(), temp_line.origin_index.end());
      }
      //长直线,直线前有复杂曲线就归档，没有复杂曲线就清空直线容器
      else {
        if (!temp_compound_curve.curve.empty()) {
          temp_compound_curve.type = port::CurveType::compound_curve;
          curves.push_back(temp_compound_curve);
          memset(&temp_compound_curve, 0, sizeof(temp_compound_curve));
        }
      }
      memset(&temp_line, 0, sizeof(temp_line));
    }

    if (line_flag) {
      temp_line.curve.push_back(smooth_path[i]);  // 累计直线点
      temp_line.origin_index.push_back(i);        // 累计直线点对应的原始路径索引
    } else {
      temp_curve.curve.push_back(smooth_path[i]);
      temp_curve.origin_index.push_back(i);
    }
    if (i == smooth_path.size() - 2 - 1) {
      //最后一段为直线
      if (!temp_line.curve.empty()) {
        /*计算直线长度*/
        line_end = temp_line.curve.back();
        line_start = temp_line.curve.front();
        line_length = sqrtf(powf(line_end.y - line_start.y, 2) + powf(line_end.x - line_start.x, 2));
        //短直线,直线归入到复杂曲线
        if (line_length < length_thd_) {
          temp_compound_curve.curve.insert(temp_compound_curve.curve.end(), temp_line.curve.begin(), temp_line.curve.end());
          temp_compound_curve.origin_index.insert(temp_compound_curve.origin_index.end(), temp_line.origin_index.begin(),
                                                  temp_line.origin_index.end());
          temp_compound_curve.type = port::CurveType::compound_curve;
          curves.push_back(temp_compound_curve);
        }
        //长直线，直线前有复杂曲线就归档，没有复杂曲线就清空直线容器
        else {
          if (!temp_compound_curve.curve.empty()) {
            temp_compound_curve.type = port::CurveType::compound_curve;
            curves.push_back(temp_compound_curve);
          }
        }
      }
      // 最后一段为曲线
      else {
        //复杂曲线未结束，曲线容器插入新的复杂曲线
        if (!temp_compound_curve.curve.empty()) {
          temp_compound_curve.curve.insert(temp_compound_curve.curve.end(), temp_curve.curve.begin(), temp_curve.curve.end());
          temp_compound_curve.origin_index.insert(temp_compound_curve.origin_index.end(), temp_curve.origin_index.begin(),
                                                  temp_curve.origin_index.end());
          temp_compound_curve.type = port::CurveType::compound_curve;
          curves.push_back(temp_compound_curve);
        }
        //曲线前无复杂曲线，根据曲线长短进行分类
        else {
          //短曲线，插入到新直线前段
          if ((int)temp_curve.curve.size() < adjust_index_) {
            temp_line = temp_curve;  // 此时新直线还未罗列点,直接将短曲线设置为直线类型
          }
          //长曲线,长曲线归档
          else {
            temp_curve.type = port::CurveType::simple_curve;
            curves.push_back(temp_curve);
          }
        }
      }
    }
    last_line_flag = line_flag;
  }
  return curves;
}

/*
 *@brief:计算急转曲线对应的约束速度
 *@param:
 *   curves:sharp curves
 *   mu:路面摩擦系数,计算最大速度对应的平衡离心力
 */
void PathClassify::CalSharpCurveV(vector<port::PathCurve>& curves) {
  float limit_v;
  float G = 9.8f;  //重力加速度
  int method = 2;
  float acc_y_max = 0.8f;
  float eps = 0.0000001f;
  //根据曲线类型,计算曲线约束线速度
  for (int i = 0; i < curves.size(); i++) {
    switch (method) {
      // method01->根据曲率计算速度(根据离心力计算,忽略拼接路径)
      case 0: {
        limit_v = sqrtf(friction_factor_ * G / fabs(curves[i].curvature));
        curves[i].limit_v = max(min_curve_v_, limit_v);  // 约束最小曲线跟踪速度,防止速度过低
        break;
      }
      // method02->根据曲率计算速度(复杂曲线设定固定跟踪速度)
      case 1: {
        if (curves[i].type == port::CurveType::simple_curve) {
          limit_v = sqrtf(friction_factor_ * G / fabs(curves[i].curvature));
          curves[i].limit_v = limit_v;
        } else {
          curves[i].limit_v = compound_curve_limit_v_;
        }
      }
      // method03->横向加速度约束
      case 2: {
        limit_v = sqrtf(acc_y_max / (fabs(curves[i].curvature) + eps));
        curves[i].limit_v = limit_v;
        curves[i].limit_v = min(1.0f, max(min_curve_v_, limit_v));
      }
      default:
        break;
    }
  }
}

/**
 *@brief:从提取出来的曲线路径进一步提取出sharp curve曲线段
 *@param:
 *   curves:原始路径上的曲线路段
 *@return:
 *   sharp_curves:急转弯曲线段
 */
vector<port::PathCurve> PathClassify::ExtractSharpCurves(vector<port::PathCurve>& curves) {
  vector<port::PathCurve> sharp_curves;
  float Ko_pc, Bo_pc, Ko_pt, Bo_pt, Xo, Yo, R, C, Theta, L;
  port::CommonPose PC, PT, PO, PCp, PTp;
  for (auto curve : curves) {
    if (curve.curve.size() >= 6) {  // todo：太短的曲线段不参与计算
      PCp = curve.curve.front();
      PTp = curve.curve.back();
      PC = curve.curve[1];
      PT = curve.curve[(curve.curve.size() - 2)];
      Ko_pc = (PCp.x - PC.x) / (PC.y - PCp.y);
      Ko_pt = (PTp.x - PT.x) / (PT.y - PTp.y);
      // !!!:临时增加方案解决空值bug
      if (PCp.x - PC.x == 0 || PTp.x - PT.x == 0 || PC.y - PCp.y == 0 || PT.y - PTp.y == 0) {
        PC = curve.curve[2];
        PT = curve.curve[(curve.curve.size() - 3)];
        Ko_pc = (PCp.x - PC.x) / (PC.y - PCp.y);
        Ko_pt = (PTp.x - PT.x) / (PT.y - PTp.y);
      }
      Bo_pc = PC.y - PC.x * Ko_pc;
      Bo_pt = PT.y - PT.x * Ko_pt;
      Xo = (Bo_pt - Bo_pc) / (Ko_pc - Ko_pt);
      Yo = Ko_pc * Xo + Bo_pc;
      R = sqrtf(powf((PC.x - Xo), 2) + powf((PC.y - Yo), 2));
      C = sqrtf(powf((PT.x - PC.x), 2) + powf((PT.y - PC.y), 2));
      //!!!:计算方式存在一定问题，反三角函数值可能有问题
      curve.theta = 2 * asinf(C / (2 * R)) * 180 / M_PI;
      int sign = Ko_pc - Ko_pt > 0 ? -1 : 1;
      curve.curvature = sign * 1.0 / R;
      sharp_curves.push_back(curve);

      // if (curve.type == compound_curve) {
      //   sharp_curves.push_back(curve);
      // } else {
      //   if (Theta > 0 && Theta < 180) {
      //     curve.curvature = 1.0 / R;
      //     sharp_curves.push_back(curve);
      //   }
      // }
    }
  }
  /*根据曲率计算速度*/
  CalSharpCurveV(sharp_curves);
  return sharp_curves;
}

void PathClassify::PlanningTrajectory(const vector<port::CommonPose>& path) {
  // trajectory速度预设
  traj_path_.clear();
  int point_num = path.size();
  traj_path_.resize(point_num);
  //期望路径点位姿&最大跟踪速度填值
  for (int i = 0; i < point_num; i++) {
    port::CommonPose traj_pose;
    traj_pose.x = path[i].x;
    traj_pose.y = path[i].y;
    if (i >= 0) {
      traj_pose.theta = mathTools::PointsAngle(path[i], path[i - 1]);
    }
    traj_path_[i] = port::TrajPoint(traj_pose, base::k_task_.ref_cmd.linear);
  }
  //急转弯路段速度信息提取
  for (auto curve : curves_) {
    //增加弯道前后速度缓冲区
    int start_index = max(0, curve.origin_index.front() - (int)(base::k_task_.ref_cmd.linear * base::k_ct_.out_curve_dis_thd_));
    int end_index = min(point_num, curve.origin_index.back() + (int)(base::k_task_.ref_cmd.linear * base::k_ct_.out_curve_dis_thd_));
    for (int i = start_index; i < end_index; i++) {
      traj_path_[i].ref_v = min(traj_path_[i].ref_v, curve.limit_v);
      if (i >= curve.origin_index.front() && i <= curve.origin_index.back()) {
        // traj_path_[i].K = curve.curvature;
      }
    }
  }
  //危险边界路段速度信息提取
  for (auto border : dg_borders_) {
    //增加危险边界前后速度缓冲区
    int start_index = max(0, border.origin_index.front() - (int)(base::k_task_.ref_cmd.linear * base::k_ct_.out_curve_dis_thd_));
    int end_index = min(point_num, border.origin_index.back() + (int)(base::k_task_.ref_cmd.linear * base::k_ct_.out_curve_dis_thd_));
    for (int i = start_index; i < end_index; i++) {
      traj_path_[i].ref_v = min(traj_path_[i].ref_v, 0.3f);
    }
  }
  //期望轨迹点相对时间计算
  for (int i = 1; i < traj_path_.size(); i++) {
    float pre_v = traj_path_[i - 1].ref_v;
    float next_v = traj_path_[i].ref_v;
    port::CommonPose pre_point = traj_path_[i - 1].pose;
    port::CommonPose next_point = traj_path_[i].pose;
    float ds = mathTools::PointsDis(next_point, pre_point);
    float dt = 2 * ds / (pre_v + next_v);
    traj_path_[i].T = traj_path_[i - 1].T + dt;
    // 计算轨迹点曲率
    float limit_v;
    float acc_y_max = 0.8f;
    float eps = 0.0000001f;
    if (i < traj_path_.size() - 2 && i > 1) {
      port::CommonPose pre_point = path[i - 2];
      port::CommonPose cur_point = path[i];
      port::CommonPose next_point = path[i + 2];
      float dis_01 = mathTools::PointsDis(cur_point, pre_point);
      float dis_02 = mathTools::PointsDis(next_point, cur_point);
      float dis_03 = mathTools::PointsDis(next_point, pre_point);
      float param_A = (cur_point.x - pre_point.x) * (next_point.y - pre_point.y);
      float param_B = (cur_point.y - pre_point.y) * (next_point.x - pre_point.x);
      float param_C = (param_A - param_B) / 2;
      traj_path_[i].K = 4 * param_C / (dis_01 * dis_02 * dis_03);
      limit_v = sqrtf(acc_y_max / (fabs(traj_path_[i].K) + eps));
      traj_path_[i].ref_v = min(next_v, max(0.2f, limit_v));
    }
  }

  if (base::VP_Instance->SpeedProfilePlanning(traj_path_, base::k_task_.ref_cmd.linear) == 0) {
    cout << "velocity QP planning success" << endl;
  } else {
    cout << "!!!velocity QP planning failure!!!" << endl;
  }
}

void PathClassify::ConvertPltPath(const vector<port::CommonPose>& path) {
  //急转弯路段可视化
  for (auto curve : curves_) {
    for (auto point_index : curve.origin_index) {
      plt_curves_.push_back(path[point_index]);
    }
  }
  //危险边界路段可视化
  for (auto border : dg_borders_) {
    for (auto point_index : border.origin_index) {
      plt_dg_borders_.push_back(path[point_index]);
    }
  }
}

void PathClassify::PathDispose(const vector<port::CommonPose>& path) {
  /***step01->特殊路径提取，可视化数据转换***/
  plt_curves_.clear();
  plt_dg_borders_.clear();
  if (path.size() > base::k_ct_.short_path_thd_) {
    //危险边界提取
    dg_borders_ = DangerousBorderClassify(path);
    //曲线提取
    auto extracted_curves = CurveClassify(path);
    curves_ = ExtractSharpCurves(extracted_curves);
    ConvertPltPath(path);
  }
  /***step02->trajectory规划***/
  PlanningTrajectory(path);
}

pair<port::DangerousBorder, port::PathCurve> PathClassify::GetProximateSpecialPath(int nearest_index) {
  /*索引需要，处理的最近的特殊路径*/
  proximate_dg_border_ = port::DangerousBorder{};
  /*还有未走过的危险边界，进行提取*/  // todo:危险边界处理
  if (!dg_borders_.empty()) {
    proximate_dg_border_ = dg_borders_.front();
    /*走出危险边界，清除掉走过的危险边界*/
    if (proximate_dg_border_.origin_index.back() + base::k_ct_.out_curve_dis_thd_ < nearest_index) {
      dg_borders_.erase(dg_borders_.begin());
    }
  }
  /*还有未走过的急转弯路段，进行提取*/
  proximate_curve_ = port::PathCurve{};
  if (!curves_.empty()) {
    proximate_curve_ = curves_.front();
    /*走出危险边界，清除掉走过的危险边界*/
    if (proximate_curve_.origin_index.back() + base::k_ct_.out_curve_dis_thd_ < nearest_index) {
      curves_.erase(curves_.begin());
    }
  }
  return make_pair(proximate_dg_border_, proximate_curve_);
}

pair<vector<port::CommonPose>, vector<port::CommonPose>> PathClassify::GetPltSpecialPath() { return make_pair(plt_dg_borders_, plt_curves_); }

vector<port::TrajPoint> PathClassify::GetPlanningTrajectory() { return traj_path_; }

}  // namespace algorithm
}  // namespace control
}  // namespace modules
