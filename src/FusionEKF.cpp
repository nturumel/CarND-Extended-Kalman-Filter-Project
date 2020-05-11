#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

 // acceleration noise for Q process noise
  const double noise_ax = 9 ;
  const double noise_ay = 9 ;
    
Eigen::VectorXd ConvertToCartesian(const VectorXd &x)
{
  VectorXd result(4);
  const double & rho=x(0);
  const double & phi=x(1);
  
  double px=rho*cos(phi);
  double py=rho*sin(phi);
  
  
  result<<px,py,0,0;
  return result; 
}

    
/**
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

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  /**
   * TODO: Finish initializing the FusionEKF.
   * TODO: Set the process and measurement noises
   */
 
    
  // H measurement noise
  H_laser_<<1,0,0,0,
  			0,1,0,0;  
  

}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {
    /**
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     * TODO: Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */

    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // TODO: Convert radar from polar to cartesian coordinates 
      //         and initialize state.
      VectorXd polar=measurement_pack.raw_measurements_;
      VectorXd cartesian=ConvertToCartesian(polar);
      ekf_.x_=cartesian;      

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // TODO: Initialize state.
	  VectorXd cartesian=measurement_pack.raw_measurements_;
	  ekf_.x_=cartesian;      
      
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */

  /**
   * TODO: Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
	
  double dt=(measurement_pack.timestamp_-previous_timestamp_);
  double dt2,dt3,dt4;
  dt2=dt*dt;
  dt3=dt2*dt;
  dt4=dt3*dt;
  ekf_.Q_=MatrixXd(4,4);
  ekf_.Q_<<(dt4*noise_ax/4),(0),(dt3*noise_ax/2),0,
  0,(dt4*noise_ay/4),0,(dt3*noise_ay/2),
  (dt3*noise_ax/2),0,(dt2*noise_ax),0,
  0,(dt3*noise_ay/2),0,(dt2*noise_ax);
  
  
  ekf_.Predict();

  /**
   * Update
   */
	
  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // TODO: Radar updates
    ekf_.R_=R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } else {
    // TODO: Laser updates
    ekf_.R_=R_laser_;
    Tools tool;
    ekf_.H_=tool.CalculateJacobian(ekf_.x_);
    ekf_.Update(measurement_pack.raw_measurements_);

  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}


