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

	cout << "Initializing UKF...\n";

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
  std_yawdd_ = 0.57;
  
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

	// Measurement noise covariance matrices
	R_laser_ = MatrixXd(2, 2);
	R_laser_ << std_laspx_ * std_laspx_, 0.0,
							0.0, std_laspy_ * std_laspy_;

	R_radar_ = MatrixXd(3, 3);
	R_radar_ << std_radr_ * std_radr_, 0.0, 0.0,
							0.0, std_radphi_ * std_radphi_, 0.0,
							0.0, 0.0, std_radrd_ * std_radrd_;

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

	// Sigma point spreading
	lambda_ = 3 - n_aug_;

	// Sigma points weights
	weights_ = VectorXd(n_sig_);
  weights_(0) = lambda_ / (lambda_ + n_aug_);
  for (unsigned int i = 1; i < 2 * n_aug_ + 1; ++i) {
    weights_(i) = 0.5 / (lambda_ + n_aug_);
  }

	cout << "Initialized UKF!\n";
}

UKF::~UKF() {}

//TODO: MIGHT NEED VECTOR ACCESSOR
void UKF::NormalizeAngle(VectorXd v, const int & i) {
	while(v(i) > M_PI) v(i) -= 2. * M_PI;
	while(v(i) < -M_PI) v(i) += 2. * M_PI;
}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {

	// CTRV model is [px, py, v, psi, psi_dot]

	cout << "Processing measurement...\n";

	if (!is_initialized_) {

		cout << "First measurement...\n";

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

		cout << "UKF has been initialized with the first measurement...\n";

		return;
	}

	double dt = (meas_package.timestamp_ - time_us_) / 1000000.0;
	time_us_ = meas_package.timestamp_;

	// Prediction step
	Prediction(dt);

	cout << "Predicted...\n";

	// Update step
	if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_)
		UpdateRadar(meas_package);
	else if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_)
		UpdateLidar(meas_package);

	cout << "Updated...\n";
}

void UKF::GenerateSigmaPoints(const VectorXd & x, const MatrixXd & P, MatrixXd & Xsig_aug) {

  VectorXd x_aug = VectorXd(7);
  MatrixXd P_aug = MatrixXd(7, 7);

  // Augmented mean state
  x_aug.head(5) = x;
  x_aug(5) = 0;
  x_aug(6) = 0;

  // Augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(5, 5) = P;
  P_aug(5, 5) = std_a_ * std_a_;
  P_aug(6, 6) = std_yawdd_ * std_yawdd_;

  // Square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  // Create augmented sigma points
  Xsig_aug.col(0) = x_aug;
  for (unsigned int i = 0; i < n_aug_; ++i) {
    Xsig_aug.col(i + 1) = x_aug + sqrt(lambda_ + n_aug_) * L.col(i);
    Xsig_aug.col(i + 1 + n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * L.col(i);
  }
}

void UKF::PredictSigmaPoints(const MatrixXd & Xsig_aug, const double & delta_t, MatrixXd & Xsig_pred) {

  for (unsigned int i = 0; i < n_sig_; ++i) {

    // Extract values for better readability
    VectorXd x = Xsig_aug.col(i).head(n_x_);
    double v = x(2);
    double psi = x(3);
    double psid = x(4);
    double nu_a = Xsig_aug.col(i)(n_x_);
    double nu_psidd = Xsig_aug.col(i)(n_x_ + 1);
    
    // Precompute some factors to avoid unnecessary repetitions
    double cos_psi = cos(psi);
    double sin_psi = sin(psi);
    double delta_t_2 = delta_t * delta_t;
    
    // Create process vector
    VectorXd x1(n_x_);
    // Avoid division by zero
    if (fabs(psid) < 0.001) {
        
        x1 << v * cos_psi * delta_t,
              v * sin_psi * delta_t,
              0, 
              0, 
              0;
    }
    else {
        
        x1 << v / psid * (sin(psi + psid * delta_t) - sin_psi),
              v / psid * (-cos(psi + psid * delta_t) + cos_psi),
              0,
              psid * delta_t,
              0;
    }

    // Create noise vector
    VectorXd x2(n_x_);
    x2 << 0.5 * delta_t_2 * cos_psi * nu_a,
          0.5 * delta_t_2 * sin_psi * nu_a,
          delta_t * nu_a,
          0.5 * delta_t_2 * nu_psidd,
          delta_t * nu_psidd;

    // Add process and noise vectors
    x += x1 + x2;
    
    Xsig_pred.col(i) = x;
  }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {

  MatrixXd Xsig_aug = MatrixXd(n_aug_, n_sig_);

  // Generate augmented sigma points
	cout << "Generating sigma points...\n";
  GenerateSigmaPoints(x_, P_, Xsig_aug);
  // Predict sigma points
	cout << "Predicting sigma points...\n";
  PredictSigmaPoints(Xsig_aug, delta_t, Xsig_pred_);

  // Predict state mean
	cout << "State mean...\n";
  x_ += Xsig_pred_ * weights_;

  // Predict state covariance matrix
	cout << "State covariance...\n";
  P_.fill(0.0);
  for (unsigned int i = 0; i < n_sig_; ++i) {
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
		//NormalizeAngle(x_diff(3));
    P_ = P_ + weights_(i) * x_diff * x_diff.transpose();
  }
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {

	cout << "Updating LASER measurement...\n";

	VectorXd z = meas_package.raw_measurements_;
	
	// Transform sigma points into measurement space
	// LIDAR measurements are [px, py]
	cout << "Transforming sigma points into measurement space...\n";
	MatrixXd Zsig = MatrixXd(2, n_sig_);
	for (unsigned int i = 0; i < n_sig_; ++i)
		Zsig.col(i) << Xsig_pred_.col(i)(0), Xsig_pred_.col(i)(1);

	// Calculate mean predicted measurement
	// TODO: optimize with vector operations
	cout << "Calculating mean predicted measurement...\n";
	VectorXd z_pred = VectorXd(2);
	z_pred.fill(0.0);
	for (unsigned int i = 0; i < n_sig_; ++i)
		z_pred += weights_(i) * Zsig.col(i);

	// Fill measurement covariance matrix and cross-correlation matrix
	cout << "Filling measurement covariance matrix...\n";
	MatrixXd S = MatrixXd(2, 2);
	S.fill(0.0);

	for (int i = 0; i < n_sig_; ++i) {
		VectorXd z_diff = Zsig.col(i) - z_pred;
		S += weights_(i) * z_diff * z_diff.transpose();
	}

	cout << "Filling cross-correlation matrix...\n";
	MatrixXd Tc = MatrixXd(n_x_, 2);
	Tc.fill(0.0);

	for (int i = 0; i < n_sig_; ++i) {
		VectorXd z_diff = Zsig.col(i) - z_pred;
		VectorXd x_diff = Xsig_pred_.col(i) - x_;
		Tc += weights_(i) * x_diff * z_diff.transpose();
	}

	// Add measurement noise covariance matrix
	S = S + R_laser_;

	// Kalman gain
	MatrixXd K = Tc * S.inverse();

	// Residual
	// TODO: Maybe we only need normalization for RADAR
	VectorXd z_diff = z - z_pred;

	// Update state mean and covariance matrix
	x_ = x_ + K * z_diff;
	P_ = P_ - K * S * K.transpose();
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {

	cout << "Updating RADAR measurement...\n";

	VectorXd z = meas_package.raw_measurements_;

	// Transform sigma points into measurement space
	// RADAR measurements are [rho, phi, and rho_dot]
	cout << "Transforming sigma points into measurement space...\n";
	MatrixXd Zsig = MatrixXd(3, n_sig_);
	for (unsigned int i = 0; i < n_sig_; ++i) {

		double px = Xsig_pred_(0, i);
		double py = Xsig_pred_(1, i);
		double v = Xsig_pred_(2, i);
		double psi = Xsig_pred_(3, i);

		double v1 = v * cos(psi);
		double v2 = v * sin(psi);
		double rho = sqrt(px * px + py * py);

		Zsig.col(i) << rho,
									 atan2(py, px),
									 (px * v1 + py * v2) / rho;
	}

	// Calculate mean predicted measurement
	// TODO: optimize with vector operations
	cout << "Calculating mean predicted measurement...\n";
	VectorXd z_pred = VectorXd(3);
	z_pred.fill(0.0);
	for (unsigned int i = 0; i < n_sig_; ++i)
		z_pred += weights_(i) * Zsig.col(i);

	// Fill measurement covariance matrix and cross-correlation matrix
	cout << "Filling measurement covariance matrix...\n";

	MatrixXd S = MatrixXd(3, 3);
	S.fill(0.0);

	for (unsigned int i = 0; i < n_sig_; ++i) {
		VectorXd z_diff = Zsig.col(i) - z_pred;
		NormalizeAngle(z_diff, 1);
		S += weights_(i) * z_diff * z_diff.transpose();
	}

	cout << "Filling cross-correlation matrix...\n";

	MatrixXd Tc = MatrixXd(n_x_, 3);
	Tc.fill(0.0);

	for (unsigned int i = 0; i < n_sig_; ++i) {
		cout << i << " " << n_sig_ << "\n";
		VectorXd z_diff = Zsig.col(i) - z_pred;
		NormalizeAngle(z_diff, 1);

		VectorXd x_diff = Xsig_pred_.col(i) - x_;
		NormalizeAngle(x_diff, 3);

		Tc += weights_(i) * x_diff * z_diff.transpose();
	}

	// Add measurement noise covariance matrix
	S = S + R_radar_;

	// Kalman gain
	MatrixXd K = Tc * S.inverse();

	// Residual
	VectorXd z_diff = z - z_pred;
	NormalizeAngle(z_diff, 1);

	// Update state mean and covariance matrix
	x_ = x_ + K * z_diff;
	P_ = P_ - K * S * K.transpose();

}
