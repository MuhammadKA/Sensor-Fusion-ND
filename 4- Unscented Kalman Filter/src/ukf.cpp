#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>
using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Initializes Unscented Kalman filter (UKF)
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);
  x_.fill(0.0);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);
  P_ << 1, 0, 0, 0, 0, 0, 0.5, 0, 0, 0, 0, 0, .25, 0, 0, 0, 0, 0, 0.25, 0, 0, 0,
      0, 0, .25;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  // tune it until you get reasonalbe results
  std_a_ = 1;

  // Process noise standard deviation yaw acceleration in rad/s^2
  // tune it until you get reasonalbe results
  std_yawdd_ = 2;

  /**
   * DO NOT MODIFY measurement noise values below.
   * These are provided by the sensor manufacturer.
   */

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;

  /**
   * End DO NOT MODIFY section for measurement noise values
   */

  /**
   * TODO: Complete the initialization. See ukf.h for other member properties.
   * Hint: one or more values initialized above might be wildly off...
   */

  // initially set to false, set to true in first call of ProcessMeasurement
  is_initialized_ = false;

  // time when the state is true, in us
  time_us_ = 0.0;

  // State dimension
  n_x_ = 5;

  // Augmented state dimension
  n_aug_ = n_x_ + 2;

  // Lambda parameter
  lambda_ = 3 - n_aug_;

  // Predicted sigma points matrix (5x15)
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);
  Xsig_pred_.fill(0.0);

  // Weights of sigma points
  weights_ = VectorXd(2 * n_aug_ + 1);

  // Initailizing the weights vectors
  weights_.fill(1 / (2 * (lambda_ + n_aug_)));
  weights_(0) = lambda_ / (lambda_ + n_aug_);
}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Make sure you switch between lidar and radar
   * measurements.
   */
  if (!is_initialized_) {

    if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      x_ << meas_package.raw_measurements_[0],
          meas_package.raw_measurements_[1], 0, 0, 0;
    }

    else if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      double rho = meas_package.raw_measurements_[0];
      double phi = meas_package.raw_measurements_[1];
      double rho_dot = meas_package.raw_measurements_[2];
      x_ << rho * cos(phi), rho * sin(phi), 0, 0, 0;
    }
    time_us_ = meas_package.timestamp_;
    is_initialized_ = true;
    std::cout << "Initialization Done." << std::endl;
    std::cout << "x: " << x_ << std::endl;
    return;
  }

  // calculate delta t
  double delta_t =
      static_cast<double>((meas_package.timestamp_ - time_us_) * 1e-6);

  // Update the recorded time for the next sensor reading
  time_us_ = meas_package.timestamp_;

  // UKF prediction funtion
  std::cout << "Prediction .." << std::endl;
  Prediction(delta_t);
  std::cout << "Prediction Done" << std::endl;
  std::cout << "Predicted X: " << x_ << std::endl;

  // UKF updata
  // This is the first time that we really need to know the measurement values
  if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
    std::cout << "Lidar Update .." << std::endl;
    UpdateLidar(meas_package);
  } else if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    std::cout << "Radar Update .." << std::endl;
    UpdateRadar(meas_package);
  }
  std::cout << "Updated X: " << x_ << std::endl;
}

void UKF::Prediction(double delta_t) {

  /**
   * TODO: Complete this function! Estimate the object's location.
   * Modify the state vector, x_. Predict sigma points, the state,
   * and the state covariance matrix.
   */

  // Preparing sigma points to represent uncertainty of covariance matrix and
  // taking into consideration the non-linear effect of noise components

  VectorXd X_aug = VectorXd::Zero(n_aug_);
  X_aug.head(5) = x_;
  X_aug(5) = 0;
  X_aug(6) = 0;

  MatrixXd P_aug = MatrixXd::Zero(n_aug_, n_aug_);
  P_aug.fill(0.0);
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug(5, 5) = std_a_ * std_a_;
  P_aug(6, 6) = std_yawdd_ * std_yawdd_;
  MatrixXd P_aug_sqrt = P_aug.llt().matrixL();

  MatrixXd Xsig_aug_ = MatrixXd::Zero(n_aug_, 2 * n_aug_ + 1);
  Xsig_aug_.col(0) = X_aug;

  for (int i = 0; i < n_aug_; i++) {
    Xsig_aug_.col(i + 1) = X_aug + sqrt(lambda_ + n_aug_) * P_aug_sqrt.col(i);
    Xsig_aug_.col(i + 1 + n_aug_) =
        X_aug - sqrt(lambda_ + n_aug_) * P_aug_sqrt.col(i);
  }

  // Sigma points prediction
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {

    // exact values from Xsig_aug_ matrix
    double px = Xsig_aug_(0, i);
    double py = Xsig_aug_(1, i);
    double v = Xsig_aug_(2, i);
    double yaw = Xsig_aug_(3, i);
    double yawd = Xsig_aug_(4, i);
    double nu_a = Xsig_aug_(5, i);
    double nu_yawdd = Xsig_aug_(6, i);

    double px_p, py_p, v_p, yaw_p, yawd_p;

    if (fabs(yawd) > 0.001) { // Curve motion
      px_p = px + v / yawd * (sin(yaw + yawd * delta_t) - sin(yaw)) +
             0.5 * delta_t * delta_t * cos(yaw) * nu_a;
      py_p = py + v / yawd * (-cos(yaw + yawd * delta_t) + cos(yaw)) +
             0.5 * delta_t * delta_t * sin(yaw) * nu_a;
      v_p = v + delta_t * nu_a;
      yaw_p = yaw + yawd * delta_t + 0.5 * delta_t * delta_t * nu_yawdd;
      yawd_p = yawd + delta_t * nu_yawdd;
    } else {
      px_p = px + v * cos(yaw) * delta_t +
             0.5 * delta_t * delta_t * cos(yaw) * nu_a;
      py_p = py + v * sin(yaw) * delta_t +
             0.5 * delta_t * delta_t * sin(yaw) * nu_a;
      v_p = v + delta_t * nu_a;
      yaw_p = yaw + yawd * delta_t + 0.5 * delta_t * delta_t * nu_yawdd;
      yawd_p = yawd + delta_t * nu_yawdd;
    }

    // update Xsig_pred
    Xsig_pred_(0, i) = px_p;
    Xsig_pred_(1, i) = py_p;
    Xsig_pred_(2, i) = v_p;
    Xsig_pred_(3, i) = yaw_p;
    Xsig_pred_(4, i) = yawd_p;
  }

  // predict state mean
  VectorXd x_final = VectorXd::Zero(n_x_);

  for (int i = 0; i < 2 * n_aug_ + 1; i++)
    x_final += weights_(i) * Xsig_pred_.col(i);

  MatrixXd P_final = MatrixXd::Zero(n_x_, n_x_);

  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    VectorXd diff = Xsig_pred_.col(i) - x_final;

    while (diff(3) > M_PI)
      diff(3) -= 2.0 * M_PI;
    while (diff(3) < -M_PI)
      diff(3) += 2.0 * M_PI;

    P_final += weights_(i) * diff * diff.transpose();
  }

  x_ = x_final;
  P_ = P_final;
}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use lidar data to update the belief
   * about the object's position. Modify the state vector, x_, and
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */

  // Project the predicted sigma points to the Lidar measurement space
  // (i.o.w: keep only the position info).
  int n_z = 2;
  MatrixXd Z_sig = MatrixXd::Zero(n_z, n_aug_ * 2 + 1);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    Z_sig(0, i) = Xsig_pred_(0, i); // px
    Z_sig(1, i) = Xsig_pred_(1, i); // py
  }

  // Get predicted mean and covariance from projected sigma points
  // in the sensor measurement space
  VectorXd Z_pred = VectorXd::Zero(n_z); // Predicted Mean vector
  MatrixXd S = MatrixXd::Zero(n_z, n_z); // Predicted Covariance matrix

  for (int i = 0; i < 2 * n_aug_ + 1; i++)
    Z_pred += weights_(i) * Z_sig.col(i);

  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    VectorXd diff_z = Z_sig.col(i) - Z_pred;
    S += weights_(i) * diff_z * diff_z.transpose();
  }

  // Add the effect of noise to the predicted covariance matrix
  // Note that the effect here is additive (not non-linear as in the case of
  // sigma points prediction)
  MatrixXd R = MatrixXd(n_z, n_z);
  R << std_laspx_ * std_laspx_, 0, 0, std_laspy_ * std_laspy_;
  S = S + R;

  // We have now the predicted state in the Lidar measurement space based on the
  // delta_t. Now lets update our belief based on the actual sensor
  // measurements.

  // Construct cross-correlation matrix Tc
  MatrixXd Tc = MatrixXd::Zero(n_x_, n_z);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    VectorXd diff_x = Xsig_pred_.col(i) - x_;
    while (diff_x(3) > M_PI)
      diff_x(3) -= 2.0 * M_PI;
    while (diff_x(3) < -M_PI)
      diff_x(3) += 2.0 * M_PI;

    VectorXd diff_z = Z_sig.col(i) - Z_pred;

    Tc += weights_(i) * diff_x * diff_z.transpose();
    // Dimensions: diff_x (5x1), diff_z (2x1) ==> Tc (5x2)
  }

  // Kalman gain matrix
  MatrixXd K = Tc * S.inverse();
  // Tc (5x2), S (2x2) ==> K (5x2)

  // Update state
  VectorXd z = meas_package.raw_measurements_;
  VectorXd Z_diff = z - Z_pred;

  // Update state mean
  x_ = x_ + K * Z_diff;
  // x_ (5x1), K (5x2), Z_diff (2x1)

  // Update state covariance
  P_ = P_ - K * S * K.transpose();
  // P_ (5x5), K (5x2), S (2x2), K_t (2x5)
}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use radar data to update the belief
   * about the object's position. Modify the state vector, x_, and
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */

  // Project the predicted sigma points to the Radar measurement space
  // (i.o.w: keep only the position info).
  int n_z = 3;
  MatrixXd Z_sig = MatrixXd::Zero(n_z, n_aug_ * 2 + 1);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    double px = Xsig_pred_(0, i);  // px
    double py = Xsig_pred_(1, i);  // py
    double v = Xsig_pred_(2, i);   // velocity
    double yaw = Xsig_pred_(3, i); // velocity

    double vx = v * cos(yaw);
    double vy = v * sin(yaw);

    Z_sig(0, i) = sqrt(px * px + py * py);                       // rho
    Z_sig(1, i) = atan2(py, px);                                 // phi
    Z_sig(2, i) = (px * vx + py * vy) / sqrt(px * px + py * py); // rho_dot
  }

  // Get predicted mean and covariance from projected sigma points
  // in the sensor measurement space
  VectorXd Z_pred = VectorXd::Zero(n_z); // Predicted Mean vector
  MatrixXd S = MatrixXd::Zero(n_z, n_z); // Predicted Covariance matrix

  for (int i = 0; i < 2 * n_aug_ + 1; i++)
    Z_pred += weights_(i) * Z_sig.col(i);

  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    VectorXd diff_z = Z_sig.col(i) - Z_pred;

    S += weights_(i) * diff_z * diff_z.transpose();
  }

  // Add the effect of noise to the predicted covariance matrix
  // Note that the effect here is additive (not non-linear as in the case of
  // sigma points prediction)
  MatrixXd R = MatrixXd::Zero(n_z, n_z);
  R << std_radr_ * std_radr_, 0, 0, 0, std_radphi_ * std_radphi_, 0, 0, 0,
      std_radrd_ * std_radrd_;
  S = S + R;

  // We have now the predicted state in the Lidar measurement space based on the
  // delta_t. Now lets update our belief based on the actual sensor
  // measurements.

  // Construct cross-correlation matrix Tc
  MatrixXd Tc = MatrixXd::Zero(n_x_, n_z);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    VectorXd diff_x = Xsig_pred_.col(i) - x_;
    while (diff_x(3) > M_PI)
      diff_x(3) -= 2.0 * M_PI;
    while (diff_x(3) < -M_PI)
      diff_x(3) += 2.0 * M_PI;

    VectorXd diff_z = Z_sig.col(i) - Z_pred;

    Tc += weights_(i) * diff_x * diff_z.transpose();

    // Dimensions: diff_x (5x1), diff_z (3x1) ==> Tc (5x3)
  }

  // Kalman gain matrix
  MatrixXd K = Tc * S.inverse();
  // Tc (5x3), S (3x3) ==> K (5x3)

  // Update state
  VectorXd z = meas_package.raw_measurements_;
  VectorXd Z_diff = z - Z_pred;

  // Update state mean
  x_ = x_ + K * Z_diff;
  // x_ (5x1), K (5x3), Z_diff (3x1)

  // Update state covariance
  P_ = P_ - K * S * K.transpose();
  // P_ (5x5), K (5x3), S (3x3), K_t (3x5)
}
