#pragma once

#include "Eigen/Dense"

namespace moxun_kalman{

template<typename T, size_t Dim>
class StateVec{
 public:
  using Scalar = T;
  static constexpr size_t Size = Dim;
  using VecType = Eigen::Matrix<T, Dim, 1>;

  StateVec() : data_(VecType::Zero()) {}
  explicit StateVec(const VecType& vec) : data_(vec) {}

  StateVec(const StateVec&) = default;
  StateVec& operator=(const StateVec&) = default;

  // 元素访问
  T operator[](size_t i) const { return data_(i); }
  T& operator[](size_t i)      { return data_(i); }

  const VecType& vector() const { return data_; }
  VecType& vector()             { return data_; }

  void setZero() { data_.setZero(); }

 private:
  VecType data_;
};

// Rigid Body Six Dim(pos_vec, rotate_vec)
template<typename T>
class RigidBodyState6d : public StateVec<T, 6>{
 public:
  using Base = StateVec<T, 6>;
  using VecType = typename Base::VecType;

  enum Index : size_t {
    PX = 0, PY = 1, PZ = 2,   // pos_vec
    RX = 3, RY = 4, RZ = 5    // rotate_vec
  };

  RigidBodyState6d() = default;
  explicit RigidBodyState6d(const VecType& vec) : Base(vec) {}
  // 位置访问
  T  x() const { return (*this)[PX]; }  T& x() { return (*this)[PX]; }
  T  y() const { return (*this)[PY]; }  T& y() { return (*this)[PY]; }
  T  z() const { return (*this)[PZ]; }  T& z() { return (*this)[PZ]; }
  // 旋转向量访问
  T  rx() const { return (*this)[RX]; } T& rx() { return (*this)[RX]; }
  T  ry() const { return (*this)[RY]; } T& ry() { return (*this)[RY]; }
  T  rz() const { return (*this)[RZ]; } T& rz() { return (*this)[RZ]; }
};


template<typename T, size_t Dim>
class ControlVec{
 public:
  using Scalar = T;
  static constexpr size_t Size = Dim;
  using VecType = Eigen::Matrix<T, Dim, 1>;

  ControlVec() : data_(VecType::Zero()) {}
  explicit ControlVec(const VecType& vec) : data_(vec) {}

  ControlVec(const ControlVec&) = default;
  ControlVec& operator=(const ControlVec&) = default;

  T operator[](size_t i) const { return data_(i); }
  T& operator[](size_t i)      { return data_(i); }

  const VecType& vector() const { return data_; }
  VecType& vector()             { return data_; }
  void setZero() { data_.setZero(); }
 private:
  VecType data_;
};

// control vec 6d(vec, ang_vec)
template<typename T>
class RigidBodyControl6d : public ControlVec<T, 6> {
 public:
  using Base = ControlVec<T, 6>;
  using VecType = typename Base::VecType;

  enum Index : size_t { VX = 0, VY , VZ, WX, WY, WZ };

  RigidBodyControl6d() = default;
  explicit RigidBodyControl6d(const VecType& vec) : Base(vec) {}
  T& vx() { return (*this)[VX]; } T  vx() const { return (*this)[VX]; }
  T& vy() { return (*this)[VY]; } T  vy() const { return (*this)[VY]; }
  T& vz() { return (*this)[VZ]; } T  vz() const { return (*this)[VZ]; }
  T& wx() { return (*this)[WX]; } T  wx() const { return (*this)[WX]; }
  T& wy() { return (*this)[WY]; } T  wy() const { return (*this)[WY]; }
  T& wz() { return (*this)[WZ]; } T  wz() const { return (*this)[WZ]; }
};

// control vec 6d(vec, ang_vec)
template<typename T>
class RigidBodyControl9d : public ControlVec<T, 9> {
 public:
  using Base = ControlVec<T, 9>;
  using VecType = typename Base::VecType;

  enum Index : size_t { VX = 0, VY , VZ, AX, AY, AZ, WX, WY, WZ };

  RigidBodyControl9d() = default;
  explicit RigidBodyControl9d(const VecType& vec) : Base(vec) {}
  T& vx() { return (*this)[VX]; } T  vx() const { return (*this)[VX]; }
  T& vy() { return (*this)[VY]; } T  vy() const { return (*this)[VY]; }
  T& vz() { return (*this)[VZ]; } T  vz() const { return (*this)[VZ]; }
  T& ax() { return (*this)[AX]; } T  ax() const { return (*this)[AX]; }
  T& ay() { return (*this)[AY]; } T  ay() const { return (*this)[AY]; }
  T& az() { return (*this)[AZ]; } T  az() const { return (*this)[AZ]; }
  T& wx() { return (*this)[WX]; } T  wx() const { return (*this)[WX]; }
  T& wy() { return (*this)[WY]; } T  wy() const { return (*this)[WY]; }
  T& wz() { return (*this)[WZ]; } T  wz() const { return (*this)[WZ]; }
};


}