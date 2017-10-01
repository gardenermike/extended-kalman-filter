#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);
  Eigen::MatrixXd F_ = MatrixXd(4, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  F_ << 1, 0, 1, 0,
        0, 1, 0, 1,
        0, 0, 1, 0,
        0, 0, 0, 1;

  Eigen::MatrixXd P_ = MatrixXd(4, 4);
  P_ << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1000, 0,
        0, 0, 0, 1000;

  Eigen::MatrixXd Q_ = MatrixXd(4, 4);

  Eigen::VectorXd x_ = VectorXd(4);

  ekf_ = KalmanFilter();
  ekf_.Init(x_, P_, F_, H_laser_, R_laser_, Q_);
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  if (!is_initialized_) {
    InitializeFirstMeasurement(measurement_pack);
    return;
  }

  //std::cout << "about to make prediction" << endl;
  MakePrediction(measurement_pack);
  //std::cout << "about to make update" << endl;
  MakeUpdate(measurement_pack);
}

void FusionEKF::InitializeFirstMeasurement(const MeasurementPackage &measurement_pack) {
  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
      * Initialize the state ekf_.x_ with the first measurement.
    */
    // first measurement
    cout << "Initializing FuxionEKF with first measurement" << endl;

    previous_timestamp_ = measurement_pack.timestamp_;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      // convert polar to cartesian coordinates
      float rho = measurement_pack.raw_measurements_[0];
      float phi = measurement_pack.raw_measurements_[1];
      float rho_dot = measurement_pack.raw_measurements_[2];
      float x = rho * cos(phi);
      float y = rho * sin(phi);
      float vx = rho_dot * cos(phi);
      float vy = rho_dot * sin(phi);
      ekf_.x_ << x, y, vx , vy;
      ekf_.R_ = R_radar_;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      //set the state with the initial location and zero velocity
      ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
      ekf_.R_ = R_laser_;
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;

    std::cout << "Initialized first measurement" << endl;
  }
}

void FusionEKF::MakePrediction(const MeasurementPackage &measurement_pack) {
  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
   */

  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;  //dt - expressed in seconds
  previous_timestamp_ = measurement_pack.timestamp_;

  ekf_.F_ << 1, 0, dt, 0,
             0, 1, 0, dt,
             0, 0, 1, 0,
             0, 0, 0, 1;

  SetQ(dt);

  ekf_.Predict();
}

void FusionEKF::MakeUpdate(const MeasurementPackage &measurement_pack) {
  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    std::cout << "radar" << endl;
    // Radar updates
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.H_ = Hj_;
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } else {
    std::cout << "laser" << endl;
    // Laser updates
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}

void FusionEKF::SetQ(const float &dt)
{
  float ax_degree_four_term = (pow(dt, 4) / 4) * noise_ax;
  float ax_degree_three_term = (pow(dt, 3) / 2) * noise_ax;
  float ax_degree_two_term = pow(dt, 2) * noise_ax;
  float ay_degree_four_term = (pow(dt, 4) / 4) * noise_ay;
  float ay_degree_three_term = (pow(dt, 3) / 2) * noise_ay;
  float ay_degree_two_term = pow(dt, 2) * noise_ay;

  ekf_.Q_ << ax_degree_four_term, 0.0, ax_degree_three_term, 0.0,
             0.0, ay_degree_four_term, 0.0, ay_degree_three_term,
             ax_degree_three_term, 0.0, ax_degree_two_term, 0.0,
             0.0, ay_degree_three_term, 0.0, ay_degree_two_term;
}
