#include "kf_modules.h"

namespace moxun_kalman{

Eigen::Quaterniond ExpMap(const Eigen::Vector3d& omega) {
  double theta = omega.norm();
  if (theta < 1e-8) {
    // 小角度近似
    return Eigen::Quaterniond::Identity();
  }
  Eigen::Vector3d axis = omega / theta;
  Eigen::AngleAxisd aa(theta, axis);
  return Eigen::Quaterniond(aa);
}

Eigen::Vector3d LogMap(const Eigen::Quaterniond& q) {
  Eigen::Quaterniond qn = q.normalized();
  double qw = qn.w();
  Eigen::Vector3d qv(qn.x(), qn.y(), qn.z());
  double norm_v = qv.norm();

  qw = std::clamp(qw, -1.0, 1.0);
  double angle = 2.0 * std::acos(qw);

  if (norm_v < 1e-8 || angle < 1e-8) {
    return Eigen::Vector3d::Zero();
  }
  Eigen::Vector3d axis = qv / norm_v;
  return axis * angle;
}


}