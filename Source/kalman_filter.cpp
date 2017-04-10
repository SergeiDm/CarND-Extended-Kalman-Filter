#include "kalman_filter.h"
#include <math.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Predict() {
  // The state prediction
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  // The state updating
	VectorXd z_pred = H_laser_ * x_;
	VectorXd y = z - z_pred;

  MatrixXd Ht = H_laser_.transpose();
	MatrixXd S = H_laser_ * P_ * Ht + R_laser_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	// New estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_laser_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  // The state updating
	VectorXd z_pred(3);
	float ro = sqrt(x_(0)*x_(0) + x_(1)*x_(1));
	float phi = atan2(x_(1), x_(0));
	float ro_dot = (x_(0)*x_(2)+x_(1)*x_(3))/ro;
  z_pred << ro, phi, ro_dot;

	VectorXd y = z - z_pred;
	MatrixXd Ht = Hj_.transpose();
	MatrixXd S = Hj_ * P_ * Ht + R_radar_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	// New estimate
	x_ = x_ + (K * y);

	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * Hj_) * P_;
}
