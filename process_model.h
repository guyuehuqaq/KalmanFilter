#pragma once
#include "kf_modules.h"
#include "kalman_filter_base.h"

namespace moxun_kalman{

template<typename StateType, typename ControlType>
class ProcessModel{
  using T = typename StateType::Scalar;
  static constexpr size_t StateDim = StateType::Size;
  static constexpr size_t ControlDim = ControlType::Size;
  using StateVector = typename StateType::VectorType;
  using ControlVector = typename ControlType::VectorType;

  virtual ~ProcessModel() = default;
  virtual StateType predict(const StateType& x, const ControlType& u, T dt) const = 0;
};

// 刚体过程模型, 恒加速度，恒角速度(CACW)
template<typename T>
class RigidBodyCACW : public ProcessModel<RigidBodyState6d<T>, RigidBodyControl6d<T>>{
 public:
  using State = RigidBodyState6d<T>;
  using Control = RigidBodyControl9d<T>;
  State predict(const State& x, const Control& u, T dt) const override{
    State next;
    next.x() = x.x() + u.vx() * dt + 0.5 * u.ax() * dt * dt;
    next.y() = x.y() + u.vy() * dt + 0.5 * u.ay() * dt * dt;
    next.z() = x.z() + u.vz() * dt + 0.5 * u.az() * dt * dt;

    Eigen::Vector3d r_k(x.rx(), x.ry(), x.rz());
    Eigen::Quaterniond q_k = ExpMap(r_k);
    Eigen::Vector3d dphi = Eigen::Vector3d(u.wx(), u.wy(), u.wz()) * dt;
    Eigen::Quaterniond dq = ExpMap(dphi);
    Eigen::Quaterniond q_next = q_k * dq;
    Eigen::Vector3d rotate_vec = LogMap(q_next);
    next.rx() = rotate_vec.x();
    next.ry() = rotate_vec.y();
    next.rz() = rotate_vec.z();
    return next;
  }
};
}