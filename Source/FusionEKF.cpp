#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

// Constructor
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // Initialize measurement covariance R_laser_ matrix
  ekf_.R_laser_ = MatrixXd(2, 2);
  ekf_.R_laser_ << 0.0225, 0,
                   0, 0.0225;

  // Initialize measurement covariance R_radar_ matrix
  ekf_.R_radar_ = MatrixXd(3, 3);
  ekf_.R_radar_ << 0.09, 0, 0,
                   0, 0.0009, 0,
                   0, 0, 0.09;

  // Initialize measurement H_laser_ matrix
  ekf_.H_laser_ = MatrixXd(2, 4);
  ekf_.H_laser_ << 1, 0, 0, 0,
                   0, 1, 0, 0;
  // Initialize measurement Hj_ matrix
  ekf_.Hj_ = MatrixXd(3, 4);
  ekf_.Hj_ << 0, 0, 0, 0,
              0, 0, 0, 0,
              0, 0, 0, 0;

  // Set initial transition matrix
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, 1, 0,
             0, 1, 0, 1,
             0, 0, 1, 0,
             0, 0, 0, 1;

  // Set initial covariance matrix
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1000, 0,
             0, 0, 0, 1000;

  // Set the acceleration noise components
  noise_ax = 9;
  noise_ay = 9;
}

// Destructor
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  // Initialization
  if (!is_initialized_) {
    // The state initialization
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      float ro = measurement_pack.raw_measurements_[0];
      float phi = measurement_pack.raw_measurements_[1];
      // Convert radar from polar to cartesian coordinates
      ekf_.x_(0) = ro*cos(phi);
      ekf_.x_(1) = ro*sin(phi);
    } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      ekf_.x_(0) = measurement_pack.raw_measurements_[0];
      ekf_.x_(1) = measurement_pack.raw_measurements_[1];
    }

    previous_timestamp_ = measurement_pack.timestamp_;

    is_initialized_ = true;
    return;
  }
  // Prediction
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
  if (dt == 0) {
      dt = 1e-5;
  }

	previous_timestamp_ = measurement_pack.timestamp_;

	float dt_2 = dt * dt;
	float dt_3 = dt_2 * dt;
	float dt_4 = dt_3 * dt;
  ekf_.F_(0, 2) = dt;
	ekf_.F_(1, 3) = dt;
  // Create covariance matrix
  ekf_.Q_ = MatrixXd(4, 4);
	ekf_.Q_ <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
              0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
              dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
              0, dt_3/2*noise_ay, 0, dt_2*noise_ay;

  ekf_.Predict();

  // Updating
  // Radar updates
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Calculate Jacobian matrix
    ekf_.Hj_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // Laser updates
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // Print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
