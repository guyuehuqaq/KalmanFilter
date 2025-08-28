#include <Eigen/Dense>

namespace moxun_kalman {

// Exponential map: so(3) → SO(3)
Eigen::Quaterniond ExpMap(const Eigen::Vector3d& omega);

// Logarithmic map: SO(3) → so(3)
Eigen::Vector3d LogMap(const Eigen::Quaterniond& q);


}