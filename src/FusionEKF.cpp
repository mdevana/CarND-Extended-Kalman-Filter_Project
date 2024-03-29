#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

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
  
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;
			  
  noise_ax=9.0;
  noise_ay=9.0;

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
     * Initialize the state ekf_.x_ with the first measurement.
     * Create the covariance matrix.
     * convert radar from polar to cartesian coordinates.
     */

    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;
	
	ekf_.F_ = MatrixXd(4, 4);
	ekf_.F_ << 1, 0, 1, 0,
			   0, 1, 0, 1,
               0, 0, 1, 0,
               0, 0, 0, 1;
	ekf_.P_ = MatrixXd(4, 4);
    ekf_.P_ << 1, 0, 0, 0,
               0, 1, 0, 0,
               0, 0, 1000, 0,
               0, 0, 0, 1000;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      //  Convert radar from polar to cartesian coordinates 
      //  and initialize state.
	  float rho_mea=measurement_pack.raw_measurements_[0];
	  float theta_mea=measurement_pack.raw_measurements_[1];
	  ekf_.x_ << rho_mea*cos(theta_mea), 
				 rho_mea*sin(theta_mea), 
                 0, 
                 0; 

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // TODO: Initialize state.
	  ekf_.x_ << measurement_pack.raw_measurements_[0], 
                 measurement_pack.raw_measurements_[1], 
                 0, 
                 0;

    }

    // done initializing, no need to predict or update
	previous_timestamp_=measurement_pack.timestamp_;
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */

  /**
   * Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
   float dt = (measurement_pack.timestamp_-previous_timestamp_)/1000000.0;
   previous_timestamp_=measurement_pack.timestamp_;
   
   ekf_.F_(0,2)=dt;
   ekf_.F_(1,3)=dt;
   
   float t_sqr=dt*dt;
   float t_cube=dt*dt*dt;
   float t_quad=dt*dt*dt*dt;
   
   
  
   // Calculate the Co-Variance Matrix Q from noise and timestep
   ekf_.Q_=MatrixXd(4,4);
   
   ekf_.Q_ << t_quad*noise_ax/4, 0, t_cube*noise_ax/2, 0,
            0, t_quad*noise_ay/4, 0, t_cube*noise_ay/2,
            t_cube*noise_ax/2, 0, t_sqr*noise_ax, 0,
            0, t_cube*noise_ay/2, 0, t_sqr*noise_ay;

   ekf_.Predict();
   
   
   

  /**
   * Update
   */

  /**
   * 
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
	VectorXd z=VectorXd(3);
	z<< measurement_pack.raw_measurements_[0],measurement_pack.raw_measurements_[1],measurement_pack.raw_measurements_[2];
	ekf_.R_ = R_radar_;
	Hj_ = tools.CalculateJacobian(ekf_.x_);
	ekf_.H_=Hj_;
	ekf_.UpdateEKF(z);

  } else {
    // Laser updates
	VectorXd z=VectorXd(2);
	z<< measurement_pack.raw_measurements_[0],measurement_pack.raw_measurements_[1];
	ekf_.R_ = R_laser_;
	ekf_.H_= H_laser_;
	ekf_.Update(z);

  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
