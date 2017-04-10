#ifndef FusionEKF_H_
#define FusionEKF_H_

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "kalman_filter.h"
#include "tools.h"

class FusionEKF {
public:
  // Constructor
  FusionEKF();

  // Destructor
  virtual ~FusionEKF();

  // Kalman Filter flow
  void ProcessMeasurement(const MeasurementPackage &measurement_pack);

  // Object of Kalman Filter class
  KalmanFilter ekf_;

private:
  // Check whether the tracking toolbox was initiallized or not (first measurement)
  bool is_initialized_;

  // Previous timestamp
  long long previous_timestamp_;

  // Tool object used to compute Jacobian and RMSE
  Tools tools;

  // Noise components
  float noise_ax;
  float noise_ay;
};

#endif /* FusionEKF_H_ */
