#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
	is_initialized_ = false;
	cnt_ = 0;
	previous_timestamp_ = 0;

	// initializing matrices
	R_laser_ = MatrixXd(2, 2); // Laser Measurement Noise
	R_radar_ = MatrixXd(3, 3); // Radar Measurement Covariance Matrix
	H_laser_ = MatrixXd(2, 4); // Laser Measurement Function
	Hj_ = MatrixXd(3, 4); // Linearized Radar Measurement Function
	ekf_.x_ = VectorXd(4); // State Vector (px, py, vx, vy)
	ekf_.P_ = MatrixXd(4, 4); // Process Covariance Matrix
	ekf_.F_ = MatrixXd(4, 4); // State Transition Function 
	ekf_.Q_ = MatrixXd(4, 4); // Process Noise

	//measurement covariance matrix - laser
	R_laser_ << 0.0225, 0,
		0, 0.0225;

	//measurement covariance matrix - radar
	R_radar_ << 0.09, 0, 0,
		0, 0.0009, 0,
		0, 0, 0.09;

	// Set State Transition Matrix
	ekf_.F_ << 1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;

	// Set initial State Covariance Matrix (low for position, high for velocities)
	ekf_.P_ << 1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1000, 0,
		0, 0, 0, 1000;

	// Expected Process Noise
	noise_ax = 9;
	noise_ay = 9;

	// Measurement Matrix for Lidar
	H_laser_ << 1, 0, 0, 0,
		0, 1, 0, 0;




	// Destructor.

	FusionEKF::~FusionEKF() {}

	void FusionEKF::ProcessMeasurement(const MeasurementPackage & measurement_pack) {
		// This function is executed after each measurement. It will predict the new state based on the state transition model
		// and then use the measurement to update that estimate.

		// Initialize position
		if (!is_initialized_) {
			ekf_.x_ = VectorXd(4);
			//LIDAR
			if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
				cout << "Initializing with Lidar Measurement" << endl;
				ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
			}
			//RADAR
			else {
				cout << "Initializing with Radar Measurement" << endl;
				ekf_.x_ << measurement_pack.raw_measurements_[0] * cos(measurement_pack.raw_measurements_[1]), measurement_pack.raw_measurements_[0] * sin(measurement_pack.raw_measurements_[1]), 0, 0;
			}
			previous_timestamp_ = measurement_pack.timestamp_;
			// done initializing, no need to predict or update
			is_initialized_ = true;
			return;
		}


		// Modify the F matrix so that the time is integrated
		double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
		cout << "dt: " << dt << endl;
		ekf_.F_(0, 2) = dt;
		previous_timestamp_ = measurement_pack.timestamp_;
		ekf_.F_(1, 3) = dt;


		float dt_2 = dt * dt;
		float dt_3 = dt_2 * dt;
		float dt_4 = dt_3 * dt;

		// set the process covariance matrix Q
		ekf_.Q_ << dt_4 / 4 * noise_ax, 0, dt_3 / 2 * noise_ax, 0,
			0, dt_4 / 4 * noise_ay, 0, dt_3 / 2 * noise_ay,
			dt_3 / 2 * noise_ax, 0, dt_2 * noise_ax, 0,
			0, dt_3 / 2 * noise_ay, 0, dt_2 * noise_ay;
		cout << "Q_: " << ekf_.Q_ << endl;
		ekf_.Predict();
		cout << "Predict:" << endl;
		cout << "P_ = " << ekf_.P_ << endl;
		cout << "x_ = " << ekf_.x_ << endl;
		cout << endl;

		//cout << "P_3 = " << ekf_.P_ << endl;
		VectorXd z = measurement_pack.raw_measurements_;

		// Update based on Radar Measurement (EKF)
		if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
			// TODO: Radar updates
			  //cout << "Receiving Radar update:" << endl; 
			  //cout << "Measurement: " << z << endl;
			ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
			ekf_.R_ = R_radar_;
			ekf_.UpdateEKF(z);
		}
		// Update based on Lidar Measurement (KF)
		else {
			//cout << "Receiving Lidar update:" << endl;
			//cout << "Measurement: " << z << endl;
			ekf_.H_ = H_laser_;
			ekf_.R_ = R_laser_;
			ekf_.Update(z);
		}

		// print the output
		cout << "x_ = " << ekf_.x_ << endl;
		cout << "P_ = " << ekf_.P_ << endl << endl;
	}
