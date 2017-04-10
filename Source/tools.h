#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"

class Tools {
public:
  // Constructor
  Tools();

  // Destructor
  virtual ~Tools();

  // Method to calculate RMSE
  Eigen::VectorXd CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations, const std::vector<Eigen::VectorXd> &ground_truth);

  // Method to calculate Jacobians
  Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd& x_state);

};

#endif /* TOOLS_H_ */
