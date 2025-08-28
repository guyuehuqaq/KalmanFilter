#pragma once

#include <utility>

#include "interface/track.h"
#include "kf_modules.h"
#include "kalman_filter_base.h"

namespace moxun_kalman{

template<typename StateType>
class MeasureModel {
 public:
  virtual ~MeasureModel() = default;

  virtual Eigen::VectorXd predictObservation(const StateType& x) const = 0;

  virtual Eigen::VectorXd computeResidual(
      const StateType& x,
      const Eigen::VectorXd& actual_measurement) const = 0;

  virtual Eigen::MatrixXd getMeasurementCovariance() const = 0;
};

// the measure model of rigid body track in opt
template<typename T>
class RigidBodyVisualModel : public MeasureModel<RigidBodyState6d<T>>{
 public:
  using State = RigidBodyState6d<T>;

  RigidBodyVisualModel(const std::vector<Eigen::Vector3d>& local_points,
                       moxun_point2d_3d::CamParaMap cam_para_map,
                       moxun_point2d_3d::CamPoseMap cam_pose_map)
      : local_points_(local_points),
        cam_para_map_(std::move(cam_para_map)),
        cam_pose_map_(std::move(cam_pose_map)){}

  Eigen::VectorXd predictObservation(const State& x) const override{
    // 计算刚体点的世界坐标
    Eigen::Quaterniond q = ExpMap(Eigen::Vector3d(x.rx(), x.ry(), x.rz()));
    Eigen::Vector3d t(x.x(), x.y(), x.z());
    Eigen::Vector3d points_w;
    for (auto& p: local_points_){
      Eigen::Vector3d p_w = q * p + t;

    }
  }


 private:
  std::vector<Eigen::Vector3d> local_points_;
  moxun_point2d_3d::CamParaMap cam_para_map_;
  moxun_point2d_3d::CamPoseMap cam_pose_map_;

};

}
