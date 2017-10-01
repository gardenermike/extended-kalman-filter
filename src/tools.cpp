#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  if (estimations.size() == 0) {
      std::cout << "estimation vector size must be greater than zero";
      return rmse;
  }
  if (estimations.size() != ground_truth.size()) {
      std::cout << "the estimation vector size must equal the ground truth vector size";
      return rmse;
  }

  //accumulate squared residuals
  for(int i=0; i < estimations.size(); ++i){
    rmse += (estimations[i] - ground_truth[i]).array().pow(2).matrix();
  }

  //calculate the mean
  rmse /= estimations.size();

  //take the square root
  rmse = rmse.array().sqrt().matrix();

  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  MatrixXd Hj(3,4);
  //recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // prevent division by zero
  float epsilon = 1e-4;
  if (px < epsilon && px > -epsilon && py < epsilon && py > -epsilon) {
    px = epsilon;
    py = epsilon;
  }

  //compute the Jacobian matrix
  float square_sum = pow(px, 2) + pow(py, 2);
  float sqrt_square_sum = sqrt(square_sum);
  Hj << (px / sqrt_square_sum), (py / sqrt_square_sum), 0, 0,
        (-py / square_sum), (px / square_sum), 0, 0,
        ((py * (vx * py - vy * px)) / pow(square_sum, 3.0 / 2.0)), ((px * (vy * px - vx * py)) / pow(square_sum, 3.0 / 2.0)), (px / sqrt_square_sum), (py / sqrt_square_sum);

  return Hj;
}
