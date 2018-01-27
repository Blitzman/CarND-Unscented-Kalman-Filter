#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);
	P_ << 1.0, 0.0, 0.0, 0.0, 0.0,
				0.0, 1.0, 0.0, 0.0, 0.0,
				0.0, 0.0, 1.0, 0.0, 0.0,
				0.0, 0.0, 0.0, 1.0, 0.0,
				0.0, 0.0, 0.0, 0.0, 1.0;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 1.5;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 3.0;
  
  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
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
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.

	// Initialization flag
	is_initialized_ = false;

	// State dimension
	n_x_ = x_.size();

	// Augmented state dimension
	n_aug_ = n_x_ + 2;

	// Number of sigma points
	n_sig_ = 2 * n_aug_ + 1;

	// Predicted sigma points matrix
	Xsig_pred_ = MatrixXd(n_x_, n_sig_);

	// Sigma points weights
	weights_ = VectorXd(n_sig_);

	// Sigma point spreading
	lambda_ = 3 - n_aug_;
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */

	// CTRV model is [px, py, v, psi, psi_dot]

	if (!is_initialized_) {

		double x = 0.0;
		double y = 0.0;
		double v = 0.0;
		double psi = 0.0;
		double psi_dot = 0.0;

		if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {

			double rho = meas_package.raw_measurements_(0);
			double phi = meas_package.raw_measurements_(1);
			double rho_dot = meas_package.raw_measurements_(2);

			x = rho * cos(phi);
			y = rho * sin(phi);
			double vx = rho_dot * cos(phi);
			double vy = rho_dot * sin(phi);
			v = sqrt(vx * vx + vy * vy);
			
		}
		else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {

			x = meas_package.raw_measurements_(0);
			y = meas_package.raw_measurements_(1);

			// TODO: Deal with special case initialization problems
		}

		x_ << x, y, v, psi, psi_dot;

		// Done initializing, no need to predict or update
		time_us_ = meas_package.timestamp_;
		is_initialized_ = true;
		return;
	}

	double dt = (meas_package.timestamp_ - time_us_) / 1000000.0;
	time_us_ = meas_package.timestamp_;

	// Prediction step
	Prediction(dt);

	// Update step
	if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_)
		UpdateRadar(meas_package);
	else if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_)
		UpdateLidar(meas_package);
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
}
