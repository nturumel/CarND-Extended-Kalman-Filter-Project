#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

<<<<<<< HEAD
/*
=======
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
>>>>>>> adf840f2e2ebfa8626158722f4db6a7ea43274dd
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225,    0.0,
                 0.0, 0.0225;

  //measurement covariance matrix - radar
<<<<<<< HEAD
  R_radar_ << 0.09,    0.0,  0.0,
               0.0, 0.0009,  0.0,
               0.0,    0.0, 0.09;

  // measurement matrix
  H_laser_ << 1.0, 0.0, 0.0, 0.0,
              0.0, 1.0, 0.0, 0.0;
=======
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
  
>>>>>>> adf840f2e2ebfa8626158722f4db6a7ea43274dd

  // measurement noises
  noise_ax_ = 9.0;
  noise_ay_ = 9.0;
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() = default;

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {

  if (!is_initialized_) {
    // initialization code

    cout << "the first measurement is received; "
         << "measurement is performed by the "
         << ((measurement_pack.sensor_type_ == MeasurementPackage::RADAR) ? "radar " : "lidar ")
         << "sensor"
         << endl;

    // initialize x---the first measurement
    VectorXd x = VectorXd(4);

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
<<<<<<< HEAD
=======
      // TODO: Convert radar from polar to cartesian coordinates 
      //         and initialize state.
      VectorXd polar=measurement_pack.raw_measurements_;
      VectorXd cartesian=ConvertToCartesian(polar);
      ekf_.x_=cartesian;      
>>>>>>> adf840f2e2ebfa8626158722f4db6a7ea43274dd

      // convert polar from polar to cartesian coordinate system
      double meas_rho     = measurement_pack.raw_measurements_[0];
      double meas_phi     = measurement_pack.raw_measurements_[1];
      double meas_px      = meas_rho     * cos(meas_phi);
      double meas_py      = meas_rho     * sin(meas_phi);

      // initial state in case the first measurement comes from radar sensor
      x << meas_px,
           meas_py,
                 0, // although radar gives velocity data in the form of the range rate rho dot​, a radar measurement
                 0; // does not contain enough information to determine the state variable velocities vx and vy
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
<<<<<<< HEAD
      // initial state in case the first measurement comes from lidar sensor
      x << measurement_pack.raw_measurements_[0],
           measurement_pack.raw_measurements_[1],
                                             0.0,    // we have no info about the velocity for lidar measurement
                                             0.0;
    } else {
      cerr << "unknown sensor type of measurement; skipping initialization" << endl;
      return;
=======
      // TODO: Initialize state.
	  VectorXd cartesian=measurement_pack.raw_measurements_;
	  ekf_.x_=cartesian;      
      
>>>>>>> adf840f2e2ebfa8626158722f4db6a7ea43274dd
    }

    // state covariance matrix (initial velocity is unknown, hence the level of uncertainty is high)
    MatrixXd P(4, 4);
    P << 1, 0,    0,    0,
         0, 1,    0,    0,
         0, 0, 1000,    0,
         0, 0,    0, 1000;

    // state transition matrix (initially Δt is 0)
    MatrixXd F(4, 4);
    F << 1, 0, 0, 0,
         0, 1, 0, 0,
         0, 0, 1, 0,
         0, 0, 0, 1;

    // process covariance matrix (initially Δt is 0, hence Q consists of 0's;
    //                            Eigen initializes matrices with 0's by default)
    MatrixXd Q(4, 4);

    ekf_.Init(x, P, F, H_laser_, R_laser_, R_radar_, Q);

    previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  //compute the time elapsed between the current and previous measurements
  double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
  previous_timestamp_ = measurement_pack.timestamp_;

<<<<<<< HEAD
  double dt_2 = dt   * dt;
  double dt_3 = dt_2 * dt;
  double dt_4 = dt_3 * dt;

  // modify the state transition matrix F so that the time is integrated
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  // update the process covariance matrix
  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ <<  dt_4/4*noise_ax_,              0.0, dt_3/2*noise_ax_,              0.0,
                           0.0, dt_4/4*noise_ay_,              0.0, dt_3/2*noise_ay_,
              dt_3/2*noise_ax_,              0.0,   dt_2*noise_ax_,              0.0,
                           0.0, dt_3/2*noise_ay_,              0.0,   dt_2*noise_ay_;

  // prediction step
  ekf_.Predict();
=======
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
>>>>>>> adf840f2e2ebfa8626158722f4db6a7ea43274dd

  // update step
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
<<<<<<< HEAD
    // Radar updates
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // Lidar updates
    ekf_.Update(measurement_pack.raw_measurements_);
=======
    // TODO: Radar updates
    ekf_.R_=R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } else {
    // TODO: Laser updates
    ekf_.R_=R_laser_;
    Tools tool;
    ekf_.H_=tool.CalculateJacobian(ekf_.x_);
    ekf_.Update(measurement_pack.raw_measurements_);

>>>>>>> adf840f2e2ebfa8626158722f4db6a7ea43274dd
  }

  // print the output
  std::cout << "==========" << std::endl
            << "x_ = "      << std::endl << ekf_.x_ << std::endl
            << "-----"      << std::endl
            << "P_ = "      << std::endl << ekf_.P_ << std::endl
            << "==========" << std::endl            << std::endl;
}


