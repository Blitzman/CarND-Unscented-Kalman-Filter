#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {

	// Initialize RMSE error.
	VectorXd rmse(4);
	rmse << 0, 0, 0, 0;

	// Check input validity.
	// 1) Estimation vector size not zero.
	// 2) Estimation vector and ground truth size must be the same.
	if (estimations.size() == 0) {
		cerr << "CalculateRMSE() - Error: Estimation is empty.\n";
		return rmse;
	}

	if (estimations.size() != ground_truth.size()) {
		cerr << "CalculateRMSE() - Error: Estimation and ground truth data size mismatch.\n";
		return rmse;
	}

	// Accumulate squared residuals.
	for (unsigned int i = 0; i < estimations.size(); ++i) {

		VectorXd residual = estimations[i] - ground_truth[i];
		residual = residual.array() * residual.array();
		rmse += residual;
	}

	// Mean calculation.
	rmse = rmse / estimations.size();

	// Square root calculation.
	rmse = rmse.array().sqrt();

	return rmse;
}
