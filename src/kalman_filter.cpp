#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
    * update the state by using Extended Kalman Filter equations
  */

  // convert x_ to polar coordinates rho, phi, and rho_dot
  float px = x_[0];
  float py = x_[1];
  float vx = x_[2];
  float vy = x_[3];
  float rho = sqrt(pow(px, 2) + pow(py, 2));

  // avoid division by zero
  float epsilon = 1e-4;
  if (rho < epsilon && rho > -epsilon)
    rho = epsilon;
  if (px < epsilon && px > -epsilon)
    px = epsilon;

  float phi = atan(py / px);

  // I have to go to some length to make sure that the incoming phi and existing phi are both
  // normalized to the same interval. Here I make sure phi is between -pi and pi
  while (phi > M_PI) {
    phi -= 2 * M_PI;
  }
  while (phi < -M_PI) {
    phi += 2 * M_PI;
  }
  float rho_dot = (px * vx + py * vy) / rho;


  Eigen::VectorXd z_normalized = VectorXd(3);
  float z_phi = z[1];
  // make sure the incoming phi is between -pi and pi
  while (z_phi > M_PI) {
    z_phi -= 2 * M_PI;
  }
  while (z_phi < -M_PI) {
    z_phi += 2 * M_PI;
  }
  // if the incoming angle and existing angle are near pi and -pi, we can get
  // nasty boundary effects where they are nearly (but not quite) 2pi radians apart.
  // the difference ends up being about 2 pi bigger than it should be.
  // This code checks if the distance between them is bigger than pi / 2, and if so,
  // assumes such a boundary case has happened and adds pi to the existing phi.
  if (fabs(phi - z_phi) > M_PI / 2) {
    if (phi > z_phi) {
      phi -= M_PI;
    } else {
      phi += M_PI;
    }
  }
  z_normalized << z[0], z_phi, z[2];

  Eigen::VectorXd h_x = VectorXd(3);
  h_x << rho, phi, rho_dot;

  VectorXd y = z_normalized - h_x;

  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
