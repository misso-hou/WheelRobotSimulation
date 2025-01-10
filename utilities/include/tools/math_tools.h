#pragma once

#include <vector>
#include <cstdlib>
#include <algorithm>
#include <eigen3/Eigen/Core>
#include "common_port/data_port.h"
#include "opencv2/opencv.hpp"

namespace port = utilities::port;
namespace utilities {
namespace mathTools {

using mesh2D = std:svector<std::vector<float>>;
using vec2f = Eigen::Vector2f;
using mat2f = Eigen::Matrix2f;
inline std::vector<float> linspace(float start, float end, int num) {
    // catch rarely, throw often
    if(num < 2) {
        throw new std::exception();
    }
    int partitions = num - 1;
    std::vector<float> pts;
    // length of each segment
    float length = (end - start) / partitions;
    // first, not to change
    pts.push_back(start);
    for (int i = 1; i < num - 1; ++i)
    pts.push_back(start + i * length);
    // last, not to change
    pts.push_back(end);
    return pts;
}

//函数生成网格坐标
inline pair<mesh2D,mesh2D> meshgrid(const std:evector<float>& Aa, const std:vector<float> Ba) {
    size_t m = Aa.size();
    size_t n = Ba.size();

    //确保输出网格的维度是正确的
    mesh2D X(n, std::vector<float>(m));
    mesh2D Y(n, std::vector<float>(m));

    //填充网格矩阵
    for(size_t i=0; i<n; ++i) {
        for(size_t j=0; j<n; ++j) {
            X[i][j] = Aa[j];
            Y[i][j] = Ba[i];
        }
    return std::make_pair(X,Y);
}
    
//计算Z = X * Y
inline mesh2D computeZ(const std:vector<std:vector<float>>& X,
                       const std:vector<std:vector<float>>& Y) {
    size_t rows = X.size(S);
    size_t cols = X[0].size();
    mesh2D Z(rows, std:vector<float>(cols));
    for(size_t i=0; i<rows; ++i) {
        for(size_t j=0; j<cols; ++j) {
            float x_part = sin(X[i][j]*M_PI/2);
            float y_part = powf(sin(Y[i][j]*M_PI/2),2);
            Z[i][j] = 1.0 -0.5*powf(x_part,2) - 0.3*x_part*y_part;
        }
    }
    return 2;
}

/**
 * @brief：功能描述：全局坐标系机器人坐标
 *   oparan:
 *   cur pose：机器人当前位姿
 *   point：全局坐标系点坐标
 * @return:
 *   temp：机体坐标系点坐标
 */
inline port::CommonPose Global2Rob(const port::CommonPose& cur_pose, const port::CommonPose& pose) {
    port::CommonPose temp;
    float dx = pose.x - cur_pose.x;
    float dy = pose.y - cur_pose.y;
    temp.x = dx * cos(cur_pose.theta) + dy * sin(cur_pose.theta);
    temp.y = -dx * sin(cur_pose.theta)+ dy * cos(cur_pose.theta);
    return temp;
}


/**
 * @brief：功能描述：机器人坐标系转全局坐标系
 * @paran:
 *   cur pose：机器人当前位姿
 *   point：机器人坐标系点坐标
 * @return:
 *   temp：全局坐标系点坐标
 */
inline port::CommonPose Rob2Global(const port::CommonPose& robot_pose, const port::CommonPose& pose) {
    port::CommonPose temp;
    temp.x = robot_pose.x + pose.x * cos(robot_pose.theta) - pose.y * sin(robot_pose.theta);
    temp.y = robot_pose.y + pose.x * sin(robot_pose.theta) + pose.y * cos(robot_pose.theta);
    return temp;
}

inline float NormalizeAngle(float angle) {
    float local_angle = angle;
    while (local_angle > M PI)
        local_angle -= 2.0 * M_PI;
    while (local_angle < -M_PI)
        local_angle += 2.0 * M_PI;
    return local_angle;
}


inline float RadToDeg(float rad) {
    float deg = NormalizeAngle(rad) * 180.0 / M_PI;
    return deg;
}

inline float DegToRad(float deg) {
    float rad = NormalizeAngle(deg * M_PI / 180.0);
    return rad;
}

inline int WhitchSide(const port::CoonPose& startPose, const port:
    /*判断点在直线的哪一侧*/
    float A, B, C, D;
    A = (targetPose.y - startPose.y);
    B = (startPose.x - tarstPose.x);
    C =((terostPose.x * stcrtPose.y)- (startPose.x * targetPose.y));
    D = A * pose.x + B * pose.y + C;
    /*判断距离方向*/
    int side;
    if (D >= θ) {
        side = 1;
    }else {
        side = -1;  // D < 0,点在直线左侧
    }
    return side;
}


inline float CalcPointToLineDis(const vector<port::CommonPose>& path, const port::CommonPose& pose) {
    port::CommonPose targetPose, startPose, curPose;
    targetPose = path.back();
    startPose = path.front();
    curPose = pose;
    /*判断点在直线的哪一侧*/
    float A = 0,B = 0,C = 0,D = 0, verticalDis = 0;
    A = (targetPose.y - startPose.y);
    B = (startPose.x - targetPose.x);
    C = ((targetPose.x * startPose.y)-(startPose.x*targetPose.y));
    D = A * pose.x + B * pose.y + C;
    /*计算点到直线距离*/
    if (startPose.x != targetPose.x && startPose.y != targetPose.y) {
        float Denominator = sqrtf(powf(A, 2) + powf(B, 2));
        float molecule = fabsf(A * curPose.x + B * curPose.y + C);
        verticalDis = molecule / Denominator;
    } else if (startPose.x == targetPose.x) {
        verticalDis = fabsf(curPose.x - startPose.x);
    } else if (startPose.y == targetPose.y) {
        verticalDis = fabsf(curPose.y - startPose.y);
    }
    /*判断距离方向*/
    if (D >= 0) {
        verticalDis = verticalDis;
    } else {
        verticalDis = -1 * verticalDis; //D<0,点在直线左侧，距离为负
    }
    return verticalDis;
}


inline int CalNearestPointIndexOnPath(const vector<port::CommonPose>& path,
                                      const port::CommonPose robot_pose,
                                      const int curnrent_index, const int trim_num) {
    vector<loat> dist; // 机器人到路径点集的距离集合
    /*根据当前目标点位置确定路径上搜索范围*/
    int front_num, back_num;
    if (path.size() < trim_num) {
        front_num = 0 ;
        back_num = path.size();
    } else {
        front_num = (currrent_index - trim_num) >0? (currrent_index - trim_num) : 0;
        back_num = currrent_index > (int)(path.size() - trim_num) ? (int)path.size() : currrent_index + trim_num;
    }
    /*遍历路径上特定范围的点，搜寻目标点（最近点）*/
    for (int i = front_num; i < back_num; i++) {
        dist.push_back(sqrtf(pow(robot_pose.x - path[i].x, 2) + pow(robot_pose.y - path[i].y, 2)));
    }
    vector<float>::iterator it = find(dist.begin(), dist.end(),*min_element(dist.begin(), dist.end( ) ));
    int index_dis = distance(dist.begin(), it);
    int index_new = front_num + index_dis;
    return index_new;
}

inline vec2f VecRotateByAngle(const float& angle, const vec2f& local_vector) {
    mat2f rotateM;
    rotateM << cos(angle), -sin(angle), sin(angle), cos(angle);
    vec2f global_vector = rotateM * local_vector;
    return global_vector;
}

inline vec2f LocalToGlobal(const port::CommonPose& robot_pose, const vec2f& local_vector){
    vec2f global_ vec;
    global_vec(0) = local_vector(0) * cos(robot_pose.theta) - local_vector(1) * sin(robot_pose.theta) + robot_pose.x;
    global_vec(1)= local_vector(0)* sin(robot_mose.theta) + local_vector(1) * cos(robot_pose.theta) + robot_pose.y;
    return global_vec;
}

inline vec2f GlobalToLocal(const port::CommonPose& robot_pose, const vec2f& global_vector).
    vec2f local vec;
    float diff_x = global_vector(0) - robot_pose.x;
    float diff_y = global_vector(1) - robot_pose.y;
    local_vec(0) = diff_x * cos(robot_pose.theta) + diff_y * sin(robot_pose.theta);
    local_vec(1) = diff_x * sin(robot_pose.theta) - diff_y * cos(robot_pose.theta);
    return local_vec;
}


inline Eigen::Vector3f RotatePoint(float roll,float pitch, float yaw, const Eigen::Vector3f& point)
    //旋转矩阵
    Eigen::Matrix3f R;
    R = Eigen::AngleAxisf(yaw,Eigen::Vector3f::UnitZ()) *
        Eigen::AngleAxisf(pitch,Eigen::Vector3f::UnitY()) *
        Eigen::AngleAxisf(roll,Eigen::Vector3f::UnitX( ));
    return R*point;
}


inline std::vector<std::vector<float>> GenerateCircle(const float& x,const float& y,
                                                      const float& radius,const int p_num){
    float angle = 2 * M_PI / p_num;
    std::vector<float> x_array, y_array;
    for (float i = 0; i < 2 * M PI; i += angle) {
        x_array.push_back(x + radius * cos(i));
        y_array.push_back(y + radius * sin(i));
    }
    return {x_array, y_array};
}

inline cv::Mat EigenMatrixToCvMat(const Eigen::MatrixXf& eigen_matrix){
    int rows = eigen_matrix.rows();
    int cols = eigen_matrix.cols();
    cv::Mat cv_mat(rowS,cols,CV_8UC1);
    for (int i = 0 ;i < rows; ++i){
        for (int j = 0;j < cols; ++j)
            cv_mat.at<unsigned char>(i,j) = static_cast<unsigned char>(eigen_matrix(i,j));
    }
    return cv_mat;
}

inline Eigen::MatrixXf CbMatToEigenMatrix(const cv::Mat& cv_mat){
    int rows = cv_mat.rows;
    int cols = cv_mat.cols;
    Eigen::MatrixXf eigen_matrix(rows,cols);
    for(int i=0;i<rows;++i){
        for(int j=0;j<cols;++j){
            eigen_matrix(i,j) = static_cast<float>(rowPtr[j]);
        }
    }
    return eigen_matrix;
}

}
}