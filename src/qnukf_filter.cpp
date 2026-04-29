#include "ros2qnukf/qnukf_filter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include <Eigen/Eigenvalues>
#include <Eigen/SVD>
#include <unsupported/Eigen/MatrixFunctions>

namespace ros2qnukf
{

namespace
{

Eigen::Quaterniond quaternion_cleanup(Eigen::Quaterniond q);

Eigen::MatrixXd matrix_pseudo_inverse(const Eigen::MatrixXd & matrix)
{
  if (matrix.rows() != matrix.cols()) {
    throw std::invalid_argument("matrix_pseudo_inverse: matrix must be square.");
  }
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
    matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
  if (svd.info() != Eigen::Success) {
    throw std::runtime_error("matrix_pseudo_inverse: SVD decomposition failed.");
  }

  const auto singular_values = svd.singularValues();
  if (singular_values.size() == 0) {
    throw std::runtime_error(
      "matrix_pseudo_inverse: Singular value vector has size zero.");
  }

  const double max_singular = singular_values.maxCoeff();
  const double tolerance =
    std::numeric_limits<double>::epsilon() *
    static_cast<double>(std::max(matrix.rows(), matrix.cols())) * max_singular;

  const double min_singular = singular_values.minCoeff();
  if (min_singular <= tolerance) {
    throw std::runtime_error("matrix_pseudo_inverse: matrix is singular or near-singular.");
  }

  Eigen::VectorXd inv_singular = Eigen::VectorXd::Zero(singular_values.size());
  for (Eigen::Index idx = 0; idx < singular_values.size(); ++idx) {
    inv_singular(idx) = 1.0 / singular_values(idx);
  }

  return svd.matrixV() * inv_singular.asDiagonal() * svd.matrixU().transpose();
}

// Mirrors pytorch3d.transforms.standardize_quaternion: unit quaternions with w >= 0.
Eigen::Quaterniond standardize_quaternion(Eigen::Quaterniond q)
{
  q.normalize();
  if (q.w() < 0.0) {
    q.coeffs() *= -1.0;
  }
  return q;
}

// DeepUKF-VIN/utils.py normalize_tensor_to_pi
Eigen::Vector3d normalize_tensor_to_pi(Eigen::Vector3d tensor)
{
  const double norm = tensor.norm();
  if (norm < 1e-15) {
    return tensor;
  }
  const double two_pi = 2.0 * M_PI;
  double wrapped = std::fmod(norm + M_PI, two_pi);
  if (wrapped < 0.0) {
    wrapped += two_pi;
  }
  wrapped -= M_PI;
  tensor *= wrapped / norm;
  return tensor;
}

// DeepUKF-VIN/utils.py qPq — quaternion_multiply(q1,q2) then normalize / R / quat / normalize / standardize.
Eigen::Quaterniond q_p_q(const Eigen::Quaterniond & q1, const Eigen::Quaterniond & q2)
{
  auto q_sum = q1 * q2;
  return quaternion_cleanup(q_sum);
}

// Rotation vector (3D) to quaternion; equivalent to axis-angle exponential map.
Eigen::Quaterniond rotation_vector_to_quaternion(const Eigen::Vector3d & rotation_vector)
{
  const double angle = rotation_vector.norm();
  if (angle < 1e-12) {
    const Eigen::Vector3d half_vec = 0.5 * rotation_vector;
    return Eigen::Quaterniond(1.0, half_vec.x(), half_vec.y(), half_vec.z()).normalized();
  }
  const Eigen::Vector3d axis = rotation_vector / angle;
  auto q = Eigen::Quaterniond(Eigen::AngleAxisd(angle, axis));
  q = quaternion_cleanup(q);
  return q;
}

// DeepUKF-VIN/utils.py qPr
Eigen::Quaterniond q_p_r(const Eigen::Quaterniond & q, const Eigen::Vector3d & r)
{
  const Eigen::Vector3d r_pi = normalize_tensor_to_pi(r);
  auto dq = rotation_vector_to_quaternion(r_pi);
  dq = quaternion_cleanup(dq);
  return q_p_q(dq, q);
}

// Quaternion to 3D rotation vector (axis * angle), matching quaternion_to_axis_angle behavior.
Eigen::Vector3d quaternion_to_rotation_vector(const Eigen::Quaterniond & q)
{
  const Eigen::Quaterniond q_clean = quaternion_cleanup(q);
  const Eigen::AngleAxisd aa{q_clean};
  return aa.angle() * aa.axis();
}

// DeepUKF-VIN/utils.py qMq = normalize_tensor_to_pi(quaternion_to_axis_angle(qPq(q1, quaternion_invert(q2))))
Eigen::Vector3d q_m_q(const Eigen::Quaterniond & q1, const Eigen::Quaterniond & q2)
{
  const Eigen::Quaterniond q_delta = q_p_q(q1, q2.conjugate());
  return normalize_tensor_to_pi(quaternion_to_rotation_vector(q_delta));
}

// Mirrors DeepUKF-VIN/six_dof_VIN.py six_dof_kin lines 43-48 after matrix_exp on q.
Eigen::Quaterniond quaternion_cleanup(Eigen::Quaterniond q)
{
  q = standardize_quaternion(q);
  const Eigen::Matrix3d rotation = q.toRotationMatrix();
  q = Eigen::Quaterniond{rotation};
  q = standardize_quaternion(q);
  return q;
}

}  // namespace

QnukfFilter::QnukfFilter()
{
  ukf_lambda_ = 3.0 - static_cast<double>(kAugmentedErrorDim);
  ukf_scaling_ = static_cast<double>(kAugmentedErrorDim) + ukf_lambda_;
  ukf_sigma_count_ = 2 * kAugmentedErrorDim + 1;
  ukf_weights_mean_ = Eigen::VectorXd::Zero(ukf_sigma_count_);
  ukf_weights_cov_ = Eigen::VectorXd::Zero(ukf_sigma_count_);
  ukf_weights_mean_(0) = ukf_lambda_ / ukf_scaling_;
  ukf_weights_cov_(0) = ukf_weights_mean_(0) + (1.0 - ukf_alpha_ * ukf_alpha_ + ukf_beta_);
  for (int idx = 1; idx < ukf_sigma_count_; ++idx) {
    ukf_weights_mean_(idx) = 1.0 / (2.0 * ukf_scaling_);
    ukf_weights_cov_(idx) = ukf_weights_mean_(idx);
  }
}

void QnukfFilter::initialize_from_pose(
  const Eigen::Vector3d & position,
  const Eigen::Quaterniond & orientation)
{
  initialized_ = true;
  orientation_ = orientation;
  orientation_.normalize();
  position_ = position;
  velocity_.setZero();
  gyro_bias_.setZero();
  accel_bias_.setZero();
  covariance_.setIdentity();
  covariance_ *= 1e-2;
  predicted_sigma_points_valid_ = false;
}

bool QnukfFilter::initialized() const noexcept
{
  return initialized_;
}

void QnukfFilter::set_debug(bool enabled) noexcept
{
  debug_logging_ = enabled;
}

void QnukfFilter::predict_batch(const std::vector<ImuMeasurement> & imu_batch)
{
  last_imu_batch_size_ = imu_batch.size();
  if (!initialized_ || imu_batch.size() < 2) {
    return;
  }

  for (std::size_t idx = 1; idx < imu_batch.size(); ++idx) {
    const auto & curr = imu_batch.at(idx);
    const auto & prev = imu_batch.at(idx - 1);
    const auto dt = (curr.stamp - prev.stamp).seconds();
    if (dt <= 1e-6) {
      continue;
    }
    predict_one_step_unscented(curr, dt);
  }
  imu_count_ += imu_batch.size();
}

void QnukfFilter::predict_step(const ImuMeasurement & prev, const ImuMeasurement & curr)
{
  last_imu_batch_size_ = 2;
  if (!initialized_) {
    return;
  }
  const auto dt = (curr.stamp - prev.stamp).seconds();
  if (dt <= 1e-6) {
    return;
  }
  predict_one_step_unscented(curr, dt);
  ++imu_count_;
}

void QnukfFilter::predict_one_step_unscented(const ImuMeasurement & current,
   const double dt)
{
  Eigen::VectorXd augmented_mean = Eigen::VectorXd::Zero(kAugmentedDim);
  augmented_mean.head<kNominalDim>() = state_to_vector(
    NominalState{orientation_, position_, velocity_, gyro_bias_, accel_bias_});

  Eigen::MatrixXd augmented_cov = Eigen::MatrixXd::Zero(kAugmentedErrorDim, kAugmentedErrorDim);
  augmented_cov.block<kErrorDim, kErrorDim>(0, 0) = covariance_;
  const double safe_dt = std::max(dt, 1e-9);
  augmented_cov.block<3, 3>(15, 15) =
    Eigen::Matrix3d::Identity() * gyro_noise_stddev_ * gyro_noise_stddev_ / safe_dt;
  augmented_cov.block<3, 3>(18, 18) =
    Eigen::Matrix3d::Identity() * accel_noise_stddev_ * accel_noise_stddev_ / safe_dt;

  const auto sqrt_cov = matrix_sqrt_psd(augmented_cov * ukf_scaling_);
  Eigen::MatrixXd sigma_augmented = Eigen::MatrixXd::Zero(kAugmentedDim, ukf_sigma_count_);
  sigma_augmented.col(0) = augmented_mean;
  for (int idx = 0; idx < kAugmentedErrorDim; ++idx) {
    const auto delta = sqrt_cov.col(idx);
    sigma_augmented.col(1 + idx) = apply_delta_to_augmented(augmented_mean, delta);
    sigma_augmented.col(1 + kAugmentedErrorDim + idx) = apply_delta_to_augmented(augmented_mean, -delta);
  }

  Eigen::MatrixXd sigma_predicted_state = Eigen::MatrixXd::Zero(kNominalDim, ukf_sigma_count_);
  for (int idx = 0; idx < ukf_sigma_count_; ++idx) {
    const auto sigma_col = sigma_augmented.col(idx);
    const auto sigma_state = state_from_vector(sigma_col.head<kNominalDim>());
    const auto imu_gyro_noise = sigma_col.segment<3>(16);
    const auto imu_accel_noise = sigma_col.segment<3>(19);
    const auto propagated = propagate_sigma_point(
      sigma_state,
      current.omega,
      current.accel,
      dt,
      imu_gyro_noise,
      imu_accel_noise,
      gravity_magnitude_);
    sigma_predicted_state.col(idx) = state_to_vector(propagated);
  }
  predicted_sigma_points_ = sigma_predicted_state;
  predicted_sigma_points_valid_ = true;

  std::vector<Eigen::Quaterniond> sigma_quats(static_cast<std::size_t>(ukf_sigma_count_));
  for (int idx = 0; idx < ukf_sigma_count_; ++idx) {
    sigma_quats[static_cast<std::size_t>(idx)] = Eigen::Quaterniond{
      sigma_predicted_state(0, idx),
      sigma_predicted_state(1, idx),
      sigma_predicted_state(2, idx),
      sigma_predicted_state(3, idx)};
  }
  const auto mean_quat = weighted_quaternion_average(sigma_quats, ukf_weights_mean_);

  Eigen::VectorXd mean_state = Eigen::VectorXd::Zero(kNominalDim);
  mean_state(0) = mean_quat.w();
  mean_state(1) = mean_quat.x();
  mean_state(2) = mean_quat.y();
  mean_state(3) = mean_quat.z();
  for (int idx = 0; idx < ukf_sigma_count_; ++idx) {
    mean_state.segment<12>(4) +=
      ukf_weights_mean_(idx) * sigma_predicted_state.col(idx).segment<12>(4);
  }

  Eigen::Matrix<double, kErrorDim, kErrorDim> predicted_cov =
    Eigen::Matrix<double, kErrorDim, kErrorDim>::Zero();
  for (int idx = 0; idx < ukf_sigma_count_; ++idx) {
    Eigen::Matrix<double, kErrorDim, 1> error =
        Eigen::Matrix<double, kErrorDim, 1>::Zero();
    const Eigen::Quaterniond sigma_quat{
      sigma_predicted_state(0, idx),
      sigma_predicted_state(1, idx),
      sigma_predicted_state(2, idx),
      sigma_predicted_state(3, idx)};
    error.segment<3>(0) = q_m_q(sigma_quat, mean_quat);
    error.segment<12>(3) =
      sigma_predicted_state.col(idx).segment<12>(4) - mean_state.segment<12>(4);
    predicted_cov += ukf_weights_cov_(idx) * (error * error.transpose());
  }
  predicted_cov.block<3, 3>(9, 9) +=
    Eigen::Matrix3d::Identity() * gyro_bias_rw_stddev_ * gyro_bias_rw_stddev_ * safe_dt;
  predicted_cov.block<3, 3>(12, 12) +=
    Eigen::Matrix3d::Identity() * accel_bias_rw_stddev_ * accel_bias_rw_stddev_ * safe_dt;

  const auto predicted_state = state_from_vector(mean_state);
  orientation_ = predicted_state.orientation;
  position_ = predicted_state.position;
  velocity_ = predicted_state.velocity;
  gyro_bias_ = predicted_state.gyro_bias;
  accel_bias_ = predicted_state.accel_bias;
  covariance_ = symmetrize(predicted_cov);
}

void QnukfFilter::update_pseudo_vision(
  const PseudoVisionMeasurement & measurement,
  const double measurement_noise_stddev)
{
  if (!initialized_) {
    return;
  }

  if (measurement.world_points.size() != measurement.body_points.size()) {
    throw std::invalid_argument("PseudoVision update requires world/body feature vectors to match in size.");
  }
  const auto feature_count = measurement.world_points.size();
  last_feature_count_ = feature_count;
  if (feature_count == 0) {
    throw std::runtime_error("PseudoVision update called with zero features.");
  }
  if (!predicted_sigma_points_valid_ || predicted_sigma_points_.cols() != ukf_sigma_count_) {
    throw std::runtime_error("Predicted sigma points are invalid or incorrectly sized in PseudoVision update.");
  }

  const auto dim_z = static_cast<int>(feature_count * 3);
  Eigen::VectorXd z = Eigen::VectorXd::Zero(dim_z);
  for (std::size_t idx = 0; idx < feature_count; ++idx) {
    const auto row = static_cast<int>(idx * 3);
    z.segment<3>(row) = measurement.body_points.at(idx);
  }

  Eigen::MatrixXd sigma_measurements = Eigen::MatrixXd::Zero(dim_z, ukf_sigma_count_);
  for (int sigma_idx = 0; sigma_idx < ukf_sigma_count_; ++sigma_idx) {
    const auto sigma_state = state_from_vector(predicted_sigma_points_.col(sigma_idx));
    const auto rotation_transpose = sigma_state.orientation.toRotationMatrix().transpose();
    for (std::size_t feature_idx = 0; feature_idx < feature_count; ++feature_idx) {
      const auto row = static_cast<int>(feature_idx * 3);
      const auto & point_world = measurement.world_points.at(feature_idx);
      sigma_measurements.block<3, 1>(row, sigma_idx) =
        rotation_transpose * (point_world - sigma_state.position);
    }
  }

  Eigen::VectorXd zhat = Eigen::VectorXd::Zero(dim_z);
  for (int sigma_idx = 0; sigma_idx < ukf_sigma_count_; ++sigma_idx) {
    zhat += ukf_weights_mean_(sigma_idx) * sigma_measurements.col(sigma_idx);
  }

  const auto sigma = measurement_noise_stddev;
  const Eigen::MatrixXd R = Eigen::MatrixXd::Identity(dim_z, dim_z) * sigma * sigma;
  Eigen::MatrixXd Pz = Eigen::MatrixXd::Zero(dim_z, dim_z);
  for (int sigma_idx = 0; sigma_idx < ukf_sigma_count_; ++sigma_idx) {
    const auto z_error = sigma_measurements.col(sigma_idx) - zhat;
    Pz += ukf_weights_cov_(sigma_idx) * (z_error * z_error.transpose());
  }
  Pz += R;

  Eigen::MatrixXd Pxz = Eigen::MatrixXd::Zero(kErrorDim, dim_z);
  const NominalState predicted_mean_state{
    orientation_,
    position_,
    velocity_,
    gyro_bias_,
    accel_bias_};
  const auto predicted_mean_vector = state_to_vector(predicted_mean_state);
  for (int sigma_idx = 0; sigma_idx < ukf_sigma_count_; ++sigma_idx) {
    const auto sigma_state = state_from_vector(predicted_sigma_points_.col(sigma_idx));
    Eigen::Matrix<double, kErrorDim, 1> x_error = Eigen::Matrix<double, kErrorDim, 1>::Zero();
    x_error.segment<3>(0) = q_m_q(sigma_state.orientation, predicted_mean_state.orientation);
    x_error.segment<12>(3) =
      predicted_sigma_points_.col(sigma_idx).segment<12>(4) - predicted_mean_vector.segment<12>(4);
    const auto z_error = sigma_measurements.col(sigma_idx) - zhat;
    Pxz += ukf_weights_cov_(sigma_idx) * (x_error * z_error.transpose());
  }

  const Eigen::MatrixXd K = Pxz * matrix_pseudo_inverse(Pz);
  const Eigen::Matrix<double, kErrorDim, 1> delta = K * (z - zhat);
  apply_error_state(delta);

  covariance_ = covariance_ - K * Pz * K.transpose();
  covariance_ = symmetrize(covariance_);
  predicted_sigma_points_valid_ = false;
  ++stereo_count_;
}

std::uint64_t QnukfFilter::imu_count() const noexcept
{
  return imu_count_;
}

std::uint64_t QnukfFilter::stereo_count() const noexcept
{
  return stereo_count_;
}

std::size_t QnukfFilter::last_imu_batch_size() const noexcept
{
  return last_imu_batch_size_;
}

std::size_t QnukfFilter::last_feature_count() const noexcept
{
  return last_feature_count_;
}

QnukfFilter::StateSnapshot QnukfFilter::state() const
{
  StateSnapshot snapshot{};
  snapshot.initialized = initialized_;
  snapshot.orientation = orientation_;
  snapshot.position = position_;
  snapshot.velocity = velocity_;
  snapshot.gyro_bias = gyro_bias_;
  snapshot.accel_bias = accel_bias_;
  return snapshot;
}

Eigen::Matrix3d QnukfFilter::skew(const Eigen::Vector3d & vector)
{
  Eigen::Matrix3d matrix = Eigen::Matrix3d::Zero();
  matrix(0, 1) = -vector.z();
  matrix(0, 2) = vector.y();
  matrix(1, 0) = vector.z();
  matrix(1, 2) = -vector.x();
  matrix(2, 0) = -vector.y();
  matrix(2, 1) = vector.x();
  return matrix;
}

Eigen::Quaterniond QnukfFilter::rot2q(const Eigen::Vector3d & rotation_vector)
{
  return rotation_vector_to_quaternion(rotation_vector);
}

Eigen::MatrixXd QnukfFilter::matrix_sqrt_psd(const Eigen::MatrixXd & matrix) const
{
  // Mirror DeepUKF-VIN/QNUKF.py pseudo_sqrt: primary path is torch.linalg.svd —
  // return U @ diag(sqrt(S)) @ Vh (real A ⇒ Vh = Vᵀ). Fallback matches pseudo_sqrt
  // eigh branch on A + reg*I.
  constexpr double kSqrtReg = 1e-10;

  if (matrix.rows() != matrix.cols()) {
    throw std::invalid_argument("matrix_sqrt_psd: matrix must be square.");
  }

  Eigen::BDCSVD<Eigen::MatrixXd> svd(matrix, Eigen::ComputeFullU | Eigen::ComputeFullV);
  if (svd.info() == Eigen::Success) {
    Eigen::VectorXd sqrt_singular = Eigen::VectorXd::Zero(svd.singularValues().size());
    for (Eigen::Index idx = 0; idx < svd.singularValues().size(); ++idx) {
      sqrt_singular(idx) = std::sqrt(std::max(0.0, svd.singularValues()(idx)));
    }
    return svd.matrixU() * sqrt_singular.asDiagonal() * svd.matrixV().transpose();
  }

  const Eigen::MatrixXd regularized = matrix + kSqrtReg * Eigen::MatrixXd::Identity(matrix.rows(), matrix.cols());
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen_solver(regularized);
  if (eigen_solver.info() != Eigen::Success) {
    throw std::runtime_error("matrix_sqrt_psd: eigendecomposition failed.");
  }

  const auto eigenvalues = eigen_solver.eigenvalues();
  if ((eigenvalues.array() < -1e-9).any()) {
    throw std::runtime_error("matrix_sqrt_psd: matrix is not positive semi-definite.");
  }
  Eigen::VectorXd sqrt_eigenvalues = Eigen::VectorXd::Zero(eigenvalues.size());
  for (Eigen::Index idx = 0; idx < eigenvalues.size(); ++idx) {
    sqrt_eigenvalues(idx) = std::sqrt(std::max(0.0, eigenvalues(idx)));
  }
  return eigen_solver.eigenvectors() * sqrt_eigenvalues.asDiagonal() *
         eigen_solver.eigenvectors().transpose();
}

Eigen::VectorXd QnukfFilter::apply_delta_to_augmented(
  const Eigen::VectorXd & augmented_state,
  const Eigen::VectorXd & delta)
{
  if (augmented_state.size() != kAugmentedDim) {
    throw std::invalid_argument("apply_delta_to_augmented: expected 22D augmented state.");
  }
  if (delta.size() != kAugmentedErrorDim) {
    throw std::invalid_argument("apply_delta_to_augmented: expected 21D delta.");
  }
  Eigen::VectorXd updated = augmented_state;
  Eigen::Quaterniond q{
    augmented_state(0), augmented_state(1), augmented_state(2), augmented_state(3)};
  q = q_p_r(q, delta.head<3>());
  updated(0) = q.w();
  updated(1) = q.x();
  updated(2) = q.y();
  updated(3) = q.z();
  updated.segment(4, 18) += delta.segment(3, 18);
  return updated;
}

QnukfFilter::NominalState QnukfFilter::state_from_vector(const Eigen::VectorXd & state_vector)
{
  if (state_vector.size() != kNominalDim) {
    throw std::invalid_argument("state_from_vector: expected 16D state vector.");
  }
  NominalState state{};
  state.orientation = Eigen::Quaterniond{
    state_vector(0), state_vector(1), state_vector(2), state_vector(3)}; //scaler first
  state.orientation.normalize();
  state.position = state_vector.segment<3>(4);
  state.velocity = state_vector.segment<3>(7);
  state.gyro_bias = state_vector.segment<3>(10);
  state.accel_bias = state_vector.segment<3>(13);
  return state;
}

Eigen::VectorXd QnukfFilter::state_to_vector(const NominalState & state)
{
  Eigen::VectorXd vector = Eigen::VectorXd::Zero(16);
  vector(0) = state.orientation.w(); //scaler first
  vector(1) = state.orientation.x();
  vector(2) = state.orientation.y();
  vector(3) = state.orientation.z();
  vector.segment<3>(4) = state.position;
  vector.segment<3>(7) = state.velocity;
  vector.segment<3>(10) = state.gyro_bias;
  vector.segment<3>(13) = state.accel_bias;
  return vector;
}

QnukfFilter::NominalState QnukfFilter::propagate_sigma_point(
  const NominalState & sigma_state,
  const Eigen::Vector3d & omega_m,
  const Eigen::Vector3d & accel_m,
  const double dt,
  const Eigen::Vector3d & imu_gyro_noise,
  const Eigen::Vector3d & imu_accel_noise,
  const double gravity_magnitude)
{
  NominalState propagated = sigma_state;
  const Eigen::Vector3d omega = omega_m - sigma_state.gyro_bias - imu_gyro_noise;
  const Eigen::Vector3d accel_body = accel_m - sigma_state.accel_bias - imu_accel_noise;

  // DeepUKF-VIN/six_dof_VIN.py six_dof_kin — same structure: 7x7 matrix_exp for [p; v; 1],
  // then 4x4 matrix_exp for quaternion, then standardize / normalize / R / quat / ...
  const Eigen::Matrix3d Rhat = sigma_state.orientation.toRotationMatrix();
  const Eigen::Vector3d e3{0.0, 0.0, 1.0};
  const Eigen::Vector3d world_accel =
    -gravity_magnitude * e3 + Rhat * accel_body;

  Eigen::Matrix<double, 7, 7> M_trans = Eigen::Matrix<double, 7, 7>::Zero();
  M_trans.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();
  M_trans.block<3, 1>(3, 6) = world_accel;

  Eigen::Matrix<double, 7, 1> X_homogeneous;
  X_homogeneous.segment<3>(0) = sigma_state.position;
  X_homogeneous.segment<3>(3) = sigma_state.velocity;
  X_homogeneous(6) = 1.0;

  const Eigen::Matrix<double, 7, 7> exp_trans = (M_trans * dt).exp();
  const Eigen::Matrix<double, 7, 1> X_new = exp_trans * X_homogeneous;
  propagated.position = X_new.segment<3>(0);
  propagated.velocity = X_new.segment<3>(3);

  Eigen::Matrix4d M_quat = Eigen::Matrix4d::Zero();
  M_quat(0, 1) = -omega.x();
  M_quat(0, 2) = -omega.y();
  M_quat(0, 3) = -omega.z();
  M_quat.block<3, 1>(1, 0) = omega;
  M_quat.block<3, 3>(1, 1) = -skew(omega);
  M_quat *= 0.5;

  Eigen::Vector4d q_col;
  q_col << sigma_state.orientation.w(), sigma_state.orientation.x(), sigma_state.orientation.y(),
    sigma_state.orientation.z();

  const Eigen::Matrix4d exp_quat = (M_quat * dt).exp();
  q_col = exp_quat * q_col;

  propagated.orientation = Eigen::Quaterniond{q_col(0), q_col(1), q_col(2), q_col(3)};
  propagated.orientation = quaternion_cleanup(propagated.orientation);
  return propagated;
}

Eigen::Quaterniond QnukfFilter::weighted_quaternion_average(
  const std::vector<Eigen::Quaterniond> & quaternions,
  const Eigen::VectorXd & weights)
{
  if (quaternions.empty()) {
    throw std::invalid_argument("weighted_quaternion_average: input quaternion vector is empty.");
  }
  if (weights.size() != static_cast<int>(quaternions.size())) {
    throw std::invalid_argument("weighted_quaternion_average: weight size mismatch.");
  }

  Eigen::Matrix4d M = Eigen::Matrix4d::Zero();
  for (int idx = 0; idx < static_cast<int>(quaternions.size()); ++idx) {
    auto q_i = quaternions.at(static_cast<std::size_t>(idx));
    q_i = standardize_quaternion(q_i);
    Eigen::Vector4d q_vec;
    q_vec << q_i.w(), q_i.x(), q_i.y(), q_i.z();
    M += weights(idx) * (q_vec * q_vec.transpose());
  }

  const double weight_sum = weights.sum();
  if (std::abs(weight_sum) < 1e-15) {
    throw std::invalid_argument("weighted_quaternion_average: weight sum is zero.");
  }
  M /= weight_sum;

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> eigensolver(M);
  if (eigensolver.info() != Eigen::Success) {
    throw std::runtime_error("weighted_quaternion_average: eigendecomposition failed.");
  }

  const Eigen::Vector4d q_avg = eigensolver.eigenvectors().col(3);  // max eigenvalue (ascending order)
  Eigen::Quaterniond q_out{q_avg(0), q_avg(1), q_avg(2), q_avg(3)};
  return quaternion_cleanup(q_out);
}

Eigen::Matrix<double, 15, 15> QnukfFilter::symmetrize(
  const Eigen::Matrix<double, 15, 15> & matrix)
{
  return 0.5 * (matrix + matrix.transpose());
}

void QnukfFilter::apply_error_state(const Eigen::Matrix<double, 15, 1> & error_state)
{
  orientation_ = q_p_r(orientation_, error_state.segment<3>(0));
  position_ += error_state.segment<3>(3);
  velocity_ += error_state.segment<3>(6);
  gyro_bias_ += error_state.segment<3>(9);
  accel_bias_ += error_state.segment<3>(12);
}

}  // namespace ros2qnukf
