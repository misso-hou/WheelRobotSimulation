#include "simulant_robot.h"

using namespace std;
using namespace cv;

namespace modules {
namespace vehicle {

void SimulantRobot::Init(const port::CommonPose& init_pose, const float& dt) {
  robot_pose_ = init_pose;
  dt_ = dt;
}

float SimulantRobot::GenerateNoise(int up_lim, int down_lim, float ratio) {
  float s = (rand() % (up_lim - down_lim + 1)) + down_lim;
  return s / ratio;
}

port::CommonPose SimulantRobot::UpdatePose(const port::Twist& cmd, int noise_pose_lim, int noise_yaw_lim) {
  static int test_count = 0;
  float s1 = 0.0f, s2 = 0.0f, s3 = 0.0f;
  if (test_count++ > 30) {
    s1 = GenerateNoise(noise_pose_lim, -noise_pose_lim, 1000.0);    // x轴噪声
    s2 = GenerateNoise(noise_pose_lim, -noise_pose_lim, 1000.0);    // y轴噪声
    s3 = GenerateNoise(noise_yaw_lim, -noise_yaw_lim, 180 / M_PI);  // 航向噪声
  }
  robot_pose_.x += cmd.linear * cos(robot_pose_.theta) * dt_ + s1;
  robot_pose_.y += cmd.linear * sin(robot_pose_.theta) * dt_ + s2;
  robot_pose_.theta += cmd.angular * dt_ + s3;
  robot_pose_.theta = atan2(sin(robot_pose_.theta), cos(robot_pose_.theta));
  return robot_pose_;
}

}  // namespace vehicle
}  // namespace modules