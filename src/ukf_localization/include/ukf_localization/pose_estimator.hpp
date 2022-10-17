#ifndef POSE_ESTIMATOR_HPP
#define POSE_ESTIMATOR_HPP

#include <memory>
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/filters/voxel_grid.h>

#include <ukf_localization/pose_system.hpp>
#include <kkl/alg/unscented_kalman_filter.hpp>

namespace ukf_localization {

/**
 * @brief scan matching-based pose estimator
 */
class PoseEstimator {
public:
  using PointT = pcl::PointXYZI;

  /**
   * @brief constructor         构造过程噪声process_noise，测量噪声measurement_noise，初始测量，协方差，时间间隔，无损卡尔曼滤波器
   * @param stamp               timestamp           当前时间戳
   * @param pos                 initial position    初始化位置
   * @param quat                initial orientation 初始化旋转
   * @param cool_time_duration  during "cool time", prediction is not performed
   */
  PoseEstimator(const ros::Time& stamp, const Eigen::Vector3f& pos, const Eigen::Quaternionf& quat, double cool_time_duration = 1.0)
    : init_stamp(stamp),
      cool_time_duration(cool_time_duration)
  {
    // 初始化过程噪声  
    process_noise = Eigen::MatrixXf::Identity(16, 16);
    process_noise.middleRows(0, 3) *= 1.0; // 位置xyz 
    process_noise.middleRows(3, 3) *= 1.0; // 速度xyz
    process_noise.middleRows(6, 4) *= 0.5; // 四元数
    process_noise.middleRows(10, 3) *= 1e-6;
    process_noise.middleRows(13, 3) *= 1e-6;

    // 初始化测量噪声 R   xyz+四元数
    Eigen::MatrixXf measurement_noise = Eigen::MatrixXf::Identity(7, 7);
    measurement_noise.middleRows(0, 3) *= 0.01;
    measurement_noise.middleRows(3, 4) *= 0.001;

    // 初始化测量
    Eigen::VectorXf mean(16);
    mean.middleRows(0, 3) = pos;
    mean.middleRows(3, 3).setZero();  // 初速度=0
    mean.middleRows(6, 4) = Eigen::Vector4f(quat.w(), quat.x(), quat.y(), quat.z());
    mean.middleRows(10, 3).setZero(); // a_bias的均值=0
    mean.middleRows(13, 3).setZero(); // g_bias的均值=0

    // 初始化协方差
    Eigen::MatrixXf cov = Eigen::MatrixXf::Identity(16, 16) * 0.01;

    PoseSystem system;  // 构造函数初始化时间间隔dt=0.01
    ukf.reset(new kkl::alg::UnscentedKalmanFilterX<float, PoseSystem>(system, 16, 6, 7, process_noise, measurement_noise, mean, cov));
  }

  /**
   * @brief predict
   * @param stamp    timestamp  点云时间戳
   * @param vel     velocity
   * @param gyro     angular velocity
   */
  void predict(const ros::Time& stamp, const Eigen::Vector3f& vel, const Eigen::Vector3f& gyro) {
    // 当前帧距离初始化没过多久 || 没有前一帧 || 前一帧和这一帧的时间是一样的（出错了）
    if((stamp - init_stamp).toSec() < cool_time_duration || prev_stamp.is_zero() || prev_stamp == stamp) {
      prev_stamp = stamp;
      return;
    }

    // 两帧数据的时间间隔
    double dt = (stamp - prev_stamp).toSec();
    prev_stamp = stamp;

    // 设置过程噪声
    ukf->setProcessNoiseCov(process_noise * dt);
    // 设置时间间隔
    ukf->system.dt = dt;

    // 控制输入
    Eigen::VectorXf control(6);
    control.head<3>() = vel;
    control.tail<3>() = gyro;

    ukf->predict(control);
  }

  void correct(float p_x,float p_y,float p_z,float q_w,float q_x,float q_y,float q_z) {

    Eigen::VectorXf observation(7);
    observation[0] = p_x;
    observation[1] = p_y;
    observation[2] = p_z;
    observation[3] = q_w;
    observation[4] = q_x;
    observation[5] = q_y;
    observation[6] = q_z;
    ukf->correct(observation);
  }

  /* getters */
  // 常函数
  Eigen::Vector3f pos() const {
    return Eigen::Vector3f(ukf->mean[0], ukf->mean[1], ukf->mean[2]);
  }

  Eigen::Vector3f vel() const {
    return Eigen::Vector3f(ukf->mean[3], ukf->mean[4], ukf->mean[5]);
  }

  Eigen::Quaternionf quat() const {
    return Eigen::Quaternionf(ukf->mean[6], ukf->mean[7], ukf->mean[8], ukf->mean[9]).normalized();
  }

  Eigen::Matrix4f matrix() const {
    Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
    m.block<3, 3>(0, 0) = quat().toRotationMatrix();
    m.block<3, 1>(0, 3) = pos();
    return m;
  }

private:
  ros::Time init_stamp;         // when the estimator was initialized
  ros::Time prev_stamp;         // when the estimator was updated last time
  double cool_time_duration;    // 冷启动时间，这段时间内不进行predict

  Eigen::MatrixXf process_noise;   // 过程噪声
  std::unique_ptr<kkl::alg::UnscentedKalmanFilterX<float, PoseSystem>> ukf; // 无损卡尔曼滤波器
};

}

#endif // POSE_ESTIMATOR_HPP
