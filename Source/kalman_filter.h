#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include "Eigen/Dense"

class KalmanFilter {
public:

  // State vector
  Eigen::VectorXd x_;

  // State covariance matrix
  Eigen::MatrixXd P_;

  // State transistion matrix
  Eigen::MatrixXd F_;

  // Process covariance matrix
  Eigen::MatrixXd Q_;

  // Measurement matrix - laser
  Eigen::MatrixXd H_laser_;

  // Measurement matrix - radar (Jacobian)
  Eigen::MatrixXd Hj_;

  // Measurement covariance matrix - laser
  Eigen::MatrixXd R_laser_;

  // Measurement covariance matrix - radar
  Eigen::MatrixXd R_radar_;

  // Constructor
  KalmanFilter();

  // Destructor
  virtual ~KalmanFilter();

  // Prediction the state and the state covariance
  void Predict();

  // Update the state by using standard Kalman Filter equations
  void Update(const Eigen::VectorXd &z);

  // Update the state by using Extended Kalman Filter equations
  void UpdateEKF(const Eigen::VectorXd &z);

};

#endif /* KALMAN_FILTER_H_ */
