#pragma once

#include <Eigen/Geometry>

namespace ros2qnukf
{

// Mirrors pytorch3d.transforms.standardize_quaternion: unit quaternions with w >= 0.
inline Eigen::Quaterniond standardize_quaternion(Eigen::Quaterniond q)
{
  q.normalize();
  if (q.w() < 0.0) {
    q.coeffs() *= -1.0;
  }
  return q;
}

// Mirrors DeepUKF-VIN/six_dof_VIN.py six_dof_kin lines 43-48 after matrix_exp on q.
inline Eigen::Quaterniond quaternion_cleanup(Eigen::Quaterniond q)
{
  q = standardize_quaternion(q);
  const Eigen::Matrix3d rotation = q.toRotationMatrix();
  q = Eigen::Quaterniond{rotation};
  q = standardize_quaternion(q);
  return q;
}

}  // namespace ros2qnukf
