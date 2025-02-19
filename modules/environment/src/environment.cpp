#include "environment.h"

#include <random>

namespace modules {
namespace env {

void Environment::InitEnv(int obs_num, const port::CommonPose& pose) {
  obs_agents_.clear();
  //设置随机数生成器
  std::random_device rd;
  std::mt19937 gen(rd());
  //设置分布范围
  std::uniform_int_distribution<> pose_x_dis(3, 10);
  std::uniform_int_distribution<> pose_y_dis(-6, 6);
  std::uniform_int_distribution<> theta_dis(-M_PI, M_PI);
  std::uniform_int_distribution<> dir_dis(0, 1);
  for (int i = 0; i < obs_num; i++) {
    Obstacle local_obs;
    switch (i) {
      case 2: {
        float local_x = pose.x + (2 * dir_dis(gen) - 1) * pose_x_dis(gen);
        float local_y = pose.y + pose_y_dis(gen);
        float local_yaw = pose.theta + theta_dis(gen);
        local_obs.SetObsProperty(ObsType::Rectangle, DataType::COUNTOUR, port::CommonPose(local_x, local_y, local_yaw), port::Twist(0.f, 0.f), 0.f,
                                 0.01f);
        local_obs.SetRectangleObsParam(5.0, 2.0, 0.1f);
        break;
      }

      case 0: {
        float local_x = pose.x + (2 * dir_dis(gen) - 1) * pose_x_dis(gen);
        float local_y = pose.y + pose_y_dis(gen);
        float local_yaw = pose.theta + theta_dis(gen);
        local_obs.SetObsProperty(ObsType::ELLIP, DataType::COUNTOUR, port::CommonPose(113.5, 51, 0.3), port::Twist(0.5f, 0.3f), 0.0f, 0.01f);
        local_obs.SetEllipsoidAxis({2.0f, 1.0f}, 15, 0.0f);
        break;
      }

      case 3: {
        float local_x = pose.x + (2 * dir_dis(gen) - 1) * pose_x_dis(gen);
        float local_y = pose.y + pose_y_dis(gen);
        float local_yaw = pose.theta + theta_dis(gen);
        local_obs.SetObsProperty(ObsType::Rectangle, DataType::COUNTOUR, port::CommonPose(local_x, local_y, local_yaw), port::Twist(0.f, 0.f), 0.f,
                                 0.01f);
        local_obs.SetRectangleObsParam(2.2, 2.2, 0.1f);
        break;
      }

      case 1: {
        float local_x = pose.x + (2 * dir_dis(gen) - 1) * pose_x_dis(gen);
        float local_y = pose.y + pose_y_dis(gen);
        float local_yaw = pose.theta + theta_dis(gen);
        local_obs.SetObsProperty(ObsType::Rectangle, DataType::COUNTOUR, port::CommonPose(114, 54, 2.1), port::Twist(0.f, 0.f), 0.f, 0.01f);
        local_obs.SetRectangleObsParam(5.0, 2.0, 0.1f);
        break;
      }

      case 4: {
        float local_x = pose.x + (2 * dir_dis(gen) - 1) * pose_x_dis(gen);
        float local_y = pose.y + pose_y_dis(gen);
        float local_yaw = pose.theta + theta_dis(gen);
        local_obs.SetObsProperty(ObsType::RANDOM, DataType::COUNTOUR, port::CommonPose(local_x, local_y, local_yaw), port::Twist(0.f, 0.f), 0.01f);
        local_obs.SetRandomObsParam(1.0f, 6, 10);
        break;
      }

      case 5: {
        float local_x = pose.x + (2 * dir_dis(gen) - 1) * pose_x_dis(gen);
        float local_y = pose.y + pose_y_dis(gen);
        float local_yaw = pose.theta + theta_dis(gen);
        local_obs.SetObsProperty(ObsType::RANDOM, DataType::COUNTOUR, port::CommonPose(local_x, local_y, local_yaw), port::Twist(0.f, 0.f), 0.01f);
        local_obs.SetRandomObsParam(1.0f, 6, 10);
        break;
      }

      case 6: {
        float local_x = pose.x + (2 * dir_dis(gen) - 1) * pose_x_dis(gen);
        float local_y = pose.y + pose_y_dis(gen);
        float local_yaw = pose.theta + theta_dis(gen);
        local_obs.SetObsProperty(ObsType::RANDOM, DataType::COUNTOUR, port::CommonPose(local_x, local_y, local_yaw), port::Twist(0.f, 0.f), 0.f);
        local_obs.SetRandomObsParam(1.0f, 6, 10);
        break;
      }

      case 7: {
        float local_x = pose.x + (2 * dir_dis(gen) - 1) * pose_x_dis(gen);
        float local_y = pose.y + pose_y_dis(gen);
        float local_yaw = pose.theta + theta_dis(gen);
        local_obs.SetObsProperty(ObsType::RANDOM, DataType::COUNTOUR, port::CommonPose(local_x, local_y, local_yaw), port::Twist(0.f, 0.f), 0.f);
        local_obs.SetRandomObsParam(1.0f, 6, 10);
        break;
      }

      case 8: {
        float local_x = pose.x + (2 * dir_dis(gen) - 1) * pose_x_dis(gen);
        float local_y = pose.y + pose_y_dis(gen);
        float local_yaw = pose.theta + theta_dis(gen);
        local_obs.SetObsProperty(ObsType::RANDOM, DataType::COUNTOUR, port::CommonPose(local_x, local_y, local_yaw), port::Twist(0.f, 0.f), 0.f);
        local_obs.SetRandomObsParam(1.0f, 6, 10);
        break;
      }

      case 9: {
        float local_x = pose.x + (2 * dir_dis(gen) - 1) * pose_x_dis(gen);
        float local_y = pose.y + pose_y_dis(gen);
        float local_yaw = pose.theta + theta_dis(gen);
        local_obs.SetObsProperty(ObsType::RANDOM, DataType::COUNTOUR, port::CommonPose(local_x, local_y, local_yaw), port::Twist(0.f, 0.f), 0.f);
        local_obs.SetRandomObsParam(1.0f, 6, 10);
        break;
      }

      case 10: {
        float local_x = pose.x + (2 * dir_dis(gen) - 1) * pose_x_dis(gen);
        float local_y = pose.y + pose_y_dis(gen);
        float local_yaw = pose.theta + theta_dis(gen);
        local_obs.SetObsProperty(ObsType::RANDOM, DataType::COUNTOUR, port::CommonPose(local_x, local_y, local_yaw), port::Twist(0.f, 0.f), 0.f);
        local_obs.SetRandomObsParam(1.0f, 6, 10);
        break;
      }

      case 11: {
        float local_x = pose.x + (2 * dir_dis(gen) - 1) * pose_x_dis(gen);
        float local_y = pose.y + pose_y_dis(gen);
        float local_yaw = pose.theta + theta_dis(gen);
        local_obs.SetObsProperty(ObsType::RANDOM, DataType::COUNTOUR, port::CommonPose(local_x, local_y, local_yaw), port::Twist(0.f, 0.f), 0.f);
        local_obs.SetRandomObsParam(1.0f, 6, 10);
        break;
      }

      default: {
        break;
      }
    }

    local_obs.ObsInit();
    obs_agents_.push_back(local_obs);
  }
}

/*障碍物位姿更新*/
void Environment::EnvUpdate() {
  show_obs_.clear();
  obs_points_cloud_.clear();
  for (auto& agent : obs_agents_) {
    agent.ObsUpdate();
    auto one_obs_show = agent.GetObsToPlot();
    show_obs_.push_back(one_obs_show);
    auto one_obs_points = agent.GetUpdatedObs();
    obs_points_cloud_.insert(obs_points_cloud_.end(), one_obs_points.begin(), one_obs_points.end());
  }
}

vector<pair<vector<float>, vector<float>>> Environment::GetShowObs() { return show_obs_; }

vector<Eigen::Vector2f> Environment::GetObsPoints() { return obs_points_cloud_; }

}  // namespace env
}  // namespace modules
