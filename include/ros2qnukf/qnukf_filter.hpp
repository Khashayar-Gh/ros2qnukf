#ifndef ROS2QNUKF__QNUKF_FILTER_HPP_
#define ROS2QNUKF__QNUKF_FILTER_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <rclcpp/time.hpp>

namespace ros2qnukf
{

class QnukfFilter
{
public:
  QnukfFilter();

  struct ImuMeasurement
  {
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    Eigen::Vector3d omega{Eigen::Vector3d::Zero()};
    Eigen::Vector3d accel{Eigen::Vector3d::Zero()};
  };

  struct PseudoVisionMeasurement
  {
    std::vector<Eigen::Vector3d> world_points{};
    std::vector<Eigen::Vector3d> body_points{};
  };

  struct StateSnapshot
  {
    bool initialized{false};
    Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
    Eigen::Vector3d gyro_bias{Eigen::Vector3d::Zero()};
    Eigen::Vector3d accel_bias{Eigen::Vector3d::Zero()};
  };

  void initialize_from_pose(
    const Eigen::Vector3d & position,
    const Eigen::Quaterniond & orientation,
    const Eigen::Vector3d & gyro_bias = Eigen::Vector3d::Zero(),
    const Eigen::Vector3d & accel_bias = Eigen::Vector3d::Zero());
  void set_process_noise_params(
    double gyro_noise_stddev,
    double accel_noise_stddev,
    double gyro_bias_rw_stddev,
    double accel_bias_rw_stddev);
  void set_initial_covariance_diagonal(
    const Eigen::Matrix<double, 15, 1> & diagonal);
  [[nodiscard]] bool initialized() const noexcept;

  void predict_batch(const std::vector<ImuMeasurement> & imu_batch);
  void predict_step(const ImuMeasurement & prev, const ImuMeasurement & curr);
  void update_pseudo_vision(const PseudoVisionMeasurement & measurement, double measurement_noise_stddev);

  [[nodiscard]] std::uint64_t imu_count() const noexcept;
  [[nodiscard]] std::uint64_t stereo_count() const noexcept;
  [[nodiscard]] std::size_t last_imu_batch_size() const noexcept;
  [[nodiscard]] std::size_t last_feature_count() const noexcept;
  [[nodiscard]] StateSnapshot state() const;

  /** Reserved diagnostics flag kept for interface compatibility. */
  void set_debug(bool enabled) noexcept;

private:
  static constexpr int kNominalDim = 16;
  static constexpr int kErrorDim = 15;
  static constexpr int kAugmentedDim = 22;
  static constexpr int kAugmentedErrorDim = 21;

  struct NominalState
  {
    Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
    Eigen::Vector3d gyro_bias{Eigen::Vector3d::Zero()};
    Eigen::Vector3d accel_bias{Eigen::Vector3d::Zero()};
  };

  static Eigen::Matrix3d skew(const Eigen::Vector3d & vector);
  static Eigen::Quaterniond rot2q(const Eigen::Vector3d & rotation_vector);
  static Eigen::Matrix<double, 15, 15> symmetrize(const Eigen::Matrix<double, 15, 15> & matrix);
  Eigen::MatrixXd matrix_sqrt_psd(const Eigen::MatrixXd & matrix) const;
  static Eigen::VectorXd apply_delta_to_augmented(
    const Eigen::VectorXd & augmented_state,
    const Eigen::VectorXd & delta);
  static NominalState state_from_vector(const Eigen::VectorXd & state_vector);
  static Eigen::VectorXd state_to_vector(const NominalState & state);
  static NominalState propagate_sigma_point(
    const NominalState & sigma_state,
    const Eigen::Vector3d & omega_m,
    const Eigen::Vector3d & accel_m,
    double dt,
    const Eigen::Vector3d & imu_gyro_noise,
    const Eigen::Vector3d & imu_accel_noise,
    double gravity_magnitude);
  static Eigen::Quaterniond weighted_quaternion_average(
    const std::vector<Eigen::Quaterniond> & quaternions,
    const Eigen::VectorXd & weights);

  void apply_error_state(const Eigen::Matrix<double, 15, 1> & error_state);
  void predict_one_step_unscented(
    const ImuMeasurement & current,
    double dt);

  std::uint64_t imu_count_{0};
  std::uint64_t stereo_count_{0};
  std::size_t last_imu_batch_size_{0};
  std::size_t last_feature_count_{0};

  bool debug_logging_{false};
  bool initialized_{false};
  Eigen::Quaterniond orientation_{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d position_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d gyro_bias_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d accel_bias_{Eigen::Vector3d::Zero()};
  Eigen::Matrix<double, 15, 15> covariance_{Eigen::Matrix<double, 15, 15>::Identity() * 1e-2};
  Eigen::Matrix<double, 15, 1> initial_covariance_diagonal_{
    (Eigen::Matrix<double, 15, 1>() <<
      1e-1, 1e-1, 1e-1,
      1e-1, 1e-1, 1e-1,
      1e-1, 1e-1, 1e-1,
      1e-1, 1e-1, 1e-1,
      1e-1, 1e-1, 1e-1).finished()};

  double gravity_magnitude_{9.81};
  // OpenVINS continuous-time defaults discretized for 200 Hz (dt = 0.005 s).
  double gyro_noise_stddev_{2.399e-3}; //augmented
  double accel_noise_stddev_{2.828e-2}; //augmented
  double gyro_bias_rw_stddev_{1.371e-6}; //additive
  double accel_bias_rw_stddev_{2.121e-4}; //additive

  double ukf_alpha_{1e-4};
  double ukf_beta_{2.0};
  double ukf_kappa_{0.0};
  double ukf_lambda_{0.0};
  double ukf_scaling_{0.0};
  int ukf_sigma_count_{0};
  Eigen::VectorXd ukf_weights_mean_{};
  Eigen::VectorXd ukf_weights_cov_{};
  Eigen::MatrixXd predicted_sigma_points_{};
  bool predicted_sigma_points_valid_{false};
};

}  // namespace ros2qnukf

#endif  // ROS2QNUKF__QNUKF_FILTER_HPP_
