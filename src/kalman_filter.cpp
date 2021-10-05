#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * This is the class where the complete Kalmin Filter equations
 * for predict and update steps are implemented. Update function  
 * is further spilt to handle Lidar and Radar cases. For Radar Case,
 * Extended Kalman filter equations are implemented. For Lidar case,
 * Linaer Kalman Filter equatons are implemented 
 */

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
  /**
   * Predict the state x Vector
   */
   x_ = F_ * x_ ;
   P_ = F_ * P_ * F_.transpose() + Q_ ;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * update the state by using Linaer Kalman Filter equations
   * Used for Lidar measurements
   */
   VectorXd y = z - H_ * x_ ;
   MatrixXd S = H_ * P_ * H_.transpose() + R_ ;
   MatrixXd K = P_ * H_.transpose() * S.inverse();
   
   long x_size = x_.size();
   MatrixXd I = MatrixXd::Identity(x_size, x_size);
      
   x_ = x_ + ( K * y) ;
   P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * update the state by using Extended Kalman Filter equations
   * used for Radar mesurements. Transform from cartesian to polar coordinates
   * Calculate Kalman gain and use it to update the correct state vector
   */
   VectorXd y = z - tools.Cartesian2Polar(x_);//transformation cartesian to polar
   y(1)=tools.NormalizeAngle(y(1));
   MatrixXd S = H_ * P_ * H_.transpose() + R_ ;
   MatrixXd K = P_ * H_.transpose() * S.inverse();
   
   long x_size = x_.size();
   MatrixXd I = MatrixXd::Identity(x_size, x_size);
      
   x_ = x_ + ( K * y) ;
   P_ = (I - K * H_) * P_;
}
