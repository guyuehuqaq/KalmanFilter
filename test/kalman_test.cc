#include <gtest/gtest.h>

#include <Eigen/Core>
#include <fstream>
#include <iostream>
#include <random>
#include <vector>

#include "kalman_filter/rigid_body_UKF.h"

#define M_PI 3.14159265358979323846
namespace moxun_kalman{
namespace {
TEST(KalmanFilter3DTest, SinusoidalMotionInXYPlane) {
  double dt = 0.01;  // 时间步长 0.01 秒
  KalmanFilter3DTrack kf(dt);
  std::ofstream log("result.csv");
  log << "time,true_x,true_y,true_z,meas_x,meas_y,meas_z,est_x,est_y,est_z\n";

  // 设置初始位置和速度
  Eigen::Vector3d init_pos(0.0, 0.0, 0.0);
  Eigen::Vector3d init_vel(0.0, 0.0, 0.0);  // 初始速度为0
  kf.setInitial(init_pos, init_vel);

  // 设置正弦波的参数
  double A = 1.0;   // y 方向的振幅
  double B = 1.0;   // z 方向的振幅
  double omega = 2 * M_PI / 20.0;  // 频率（周期为 2 秒）
  double phase = 0.0; // 相位偏移

  // 模拟噪声
  std::default_random_engine gen;
  std::normal_distribution<double> noise(0.0, 0.001);  // 测量噪声

  // 记录预测和测量数据的误差
  double total_position_error = 0.0;
  double total_velocity_error = 0.0;

  for (int frame = 0; frame < 2000; ++frame) {
    double time = frame * dt;

    // 真实位置（目标正弦波运动）
    double y_true = time;
    double z_true = B * std::sin(omega * time + phase);
    Eigen::Vector3d true_pos(0.0, y_true, z_true);

    // 模拟带噪声的观测数据
    Eigen::Vector3d meas = true_pos + Eigen::Vector3d(noise(gen), noise(gen), noise(gen));

    // 执行预测 + 更新
    kf.predict();
    kf.update(meas);

    // 获取估计的目标位置和速度
    Eigen::Vector3d est_pos = kf.getPosition();
    Eigen::Vector3d est_vel = kf.getVelocity();

    // 计算位置和速度的误差
    double position_error = (est_pos - true_pos).norm();
    double velocity_error = (est_vel).norm();  // 这里只关心速度在 xy 平面内的误差

    total_position_error += position_error;
    total_velocity_error += velocity_error;

    // 输出当前时刻的预测结果与真实值
    std::cout << "t=" << time
              << " | True: " << true_pos.transpose()
              << " | Meas: " << meas.transpose()
              << " | Est Pos: " << est_pos.transpose()
              << " | Est Vel: " << est_vel.transpose()
              << " | Pos Error: " << position_error
              << " | Vel Error: " << velocity_error << "\n";
    log << time << ","
        << true_pos.x() << "," << true_pos.y() << "," << true_pos.z() << ","
        << meas.x() << "," << meas.y() << "," << meas.z() << ","
        << est_pos.x() << "," << est_pos.y() << "," << est_pos.z() << "\n";
  }
  log.close();
  // 测试结束，检查总体误差
  std::cout << "Total Position Error: " << total_position_error << std::endl;
  std::cout << "Total Velocity Error: " << total_velocity_error << std::endl;

}



}
}