#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using std::cout;
using std::endl;
using std::endl;
Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd>& estimations,
	const vector<VectorXd>& ground_truth) {
	// This function calculates the Root Mean Square Error to evaluate the performance of the EKF
	// GOAL: 
		// Dataset 1: 0.11, 0.11, 0.52, 0.52
	// With Lidar only: 
		// Dataset 1: 0.2249, 0.2186, 3.7413, 3.3195
	// With Radar:
		// Dataset 1: 0.0973, 0.0855, 0.4513, 0.4399
		// Dataset 2: 0.0726, 0.0967, 0.4579, 0.4966


	// From Lesson 23 #24
	VectorXd rmse(4);
	rmse << 0, 0, 0, 0;

	// Checking the validity of the following inputs:
	//  * the estimation vector size should not be zero
	//  * the estimation vector size should equal ground truth vector size
	if (estimations.size() != ground_truth.size()
		|| estimations.size() == 0) {
		cout << "Invalid estimation or ground_truth data" << endl;
		return rmse;
	}
	for (unsigned int i = 0; i < estimations.size(); ++i) {
		VectorXd residual = estimations[i] - ground_truth[i];
		residual = residual.array() * residual.array();
		rmse += residual;
	}
	rmse = rmse / estimations.size();
	rmse = rmse.array().sqrt();
	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {

	// Implementation from Nanodegree Lesson 23 Part 20
	MatrixXd Jacobian(3, 4);

	// recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	// pre-compute a set of terms to avoid repeated calculation
	float c1 = px * px + py * py;
	float c2 = sqrt(c1);
	float c3 = (c1 * c2);

	// check division by zero
	if (fabs(c1) < 0.0001) {
		cout << "CalculateJacobian () - Error - Division by Zero" << endl;
		c1 = 0.0001;
	}

	// compute the Jacobian matrix
	Jacobian << (px / c2), (py / c2), 0, 0,
		-(py / c1), (px / c1), 0, 0,
		py* (vx * py - vy * px) / c3, px* (px * vy - py * vx) / c3, px / c2, py / c2;

	return Jacobian;
}
