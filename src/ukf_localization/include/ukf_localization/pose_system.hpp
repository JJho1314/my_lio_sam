#ifndef POSE_SYSTEM_HPP
#define POSE_SYSTEM_HPP

#include <kkl/alg/unscented_kalman_filter.hpp>

namespace ukf_localization {

/**
 * @brief Definition of system to be estimated by ukf
 * @note state = [px, py, pz, vx, vy, vz, qw, qx, qy, qz, acc_bias_x, acc_bias_y, acc_bias_z, gyro_bias_x, gyro_bias_y, gyro_bias_z]
 */
class PoseSystem {
public:
  typedef float T;
  typedef Eigen::Matrix<T, 3, 1> Vector3t;
  typedef Eigen::Matrix<T, 4, 4> Matrix4t;
  typedef Eigen::Matrix<T, Eigen::Dynamic, 1> VectorXt;
  typedef Eigen::Quaternion<T> Quaterniont;
public:
  PoseSystem() {
    dt = 0.01;  // TODO 时间间隔改成可以修改的
  }

  // system equation
  // 系统方程，即以现在的状态和输入预测下一刻的状态
  /**
   * @param state 当前状态
   * @param control 控制量
   */
  VectorXt f(const VectorXt& state, const VectorXt& control) const {
    VectorXt next_state(16);

    // 当前状态
    Vector3t pt = state.middleRows(0, 3);
    // Vector3t vt = state.middleRows(3, 3);
    Quaterniont qt(state[6], state[7], state[8], state[9]);
    qt.normalize();

    Vector3t acc_bias = state.middleRows(10, 3);
    Vector3t gyro_bias = state.middleRows(13, 3);

    Vector3t vt = control.middleRows(0, 3);
    Vector3t raw_gyro = control.middleRows(3, 3);

    
    // velocity
    // Vector3t g(0.0f, 0.0f, -9.80665f); // TODO 重力是否要设成可调节的
    // Vector3t acc_ = raw_acc - acc_bias+g;
    // Vector3t acc = qt * acc_;
    // position
    // next_state.middleRows(0, 3) = pt + vt * dt;				
    next_state.middleRows(0, 3) = pt + vt * dt;		
    next_state.middleRows(3, 3) = vt; 	// acceleration didn't contribute to accuracy due to large noise

    // orientation
    Vector3t gyro = raw_gyro - gyro_bias;
    Quaterniont dq(1, gyro[0] * dt / 2, gyro[1] * dt / 2, gyro[2] * dt / 2); // 四元数的更新
    dq.normalize();
    Quaterniont qt_ = (qt * dq).normalized();
    next_state.middleRows(6, 4) << qt_.w(), qt_.x(), qt_.y(), qt_.z();

    next_state.middleRows(10, 3) = state.middleRows(10, 3);		// constant bias on acceleration    // TODO可以用紧耦合优化 bias
    next_state.middleRows(13, 3) = state.middleRows(13, 3);		// constant bias on angular velocity //  TODO可以用紧耦合优化 bias

    return next_state;
  }

  // observation equation
  // 观测方程
  VectorXt h(const VectorXt& state) const {
    VectorXt observation(7);
    observation.middleRows(0, 3) = state.middleRows(0, 3);
    observation.middleRows(3, 4) = state.middleRows(6, 4).normalized();

    return observation;
  }

  double dt;  //两帧点云的时间间隔
};

}

#endif // POSE_SYSTEM_HPP
