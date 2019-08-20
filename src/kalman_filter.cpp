#include "kalman_filter.h"
#include "math.h"
#define _USE_MATH_DEFINES


using Eigen::MatrixXd;
using Eigen::VectorXd;

// Constructor
KalmanFilter::KalmanFilter() {}

// Destructor
KalmanFilter::~KalmanFilter() {}

// Initialize state vector, unsertainty and transition matrices
void KalmanFilter::Init(VectorXd& x_in, MatrixXd& P_in, MatrixXd& F_in,
	MatrixXd& H_in, MatrixXd& R_in, MatrixXd& Q_in) {
	x_ = x_in; // State Vector
	P_ = P_in; // Covariance Matrix
	F_ = F_in; // State Function
	H_ = H_in; // Measurement Function
	R_ = R_in; // Sensor Measurement Noise
	Q_ = Q_in; // Process Noise
}

// Prediction by using the State Function
void KalmanFilter::Predict() {
	// This function predicts the next state based on a constant velocity model
	x_ = F_ * x_;
	P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd& z) {
	// This function updates the position and Uncertainties based on Lidar data
	VectorXd z_pred = H_ * x_;
	VectorXd y = z - z_pred; // Error Calculation
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si; // Kalman Gain

	//new estimate
	x_ = x_ + (K * y);
	MatrixXd I = MatrixXd::Identity(4, 4);
	P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd& z) {
	// This function updates the positions and velocities based on radar data
	VectorXd z_pred = VectorXd(3);
	double p_x = x_(0);
	double p_y = x_(1);
	double v_x = x_(2);
	double v_y = x_(3);
	double p_x2 = p_x * p_x;
	double p_y2 = p_y * p_y;
	double rho = sqrt(p_x2 + p_y2);
	double phi = atan2(p_y, p_x);
	double rho_dot = (p_x * v_x + p_y * v_y) / rho;
	z_pred << rho, phi, rho_dot;
	VectorXd y = z - z_pred;
	while (y(1) > M_PI) {
		y(1) -= 2 * M_PI;
	}
	while (y(1) < -M_PI) {
		y(1) += 2 * M_PI;
	}
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
	MatrixXd I = MatrixXd::Identity(4, 4);
	P_ = (I - K * H_) * P_;

}
