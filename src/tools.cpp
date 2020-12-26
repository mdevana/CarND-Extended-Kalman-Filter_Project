#include "tools.h"
#include <iostream>
#include <math.h>


using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  // TODO: YOUR CODE HERE
  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  
  if ((estimations.size()==0) || (estimations.size()!=ground_truth.size())){
    std::cout<<"invalid inputs";
    return(rmse);
  }
    
  // TODO: accumulate squared residuals
  //cout<<estimations[0]<<endl;
  VectorXd a,b;
  VectorXd c;
  for (int i=0; i < estimations.size(); ++i) {
    
    a=estimations[i];
    b=ground_truth[i];
    c=a-b;
    c=(c.array()*c.array());
    rmse+=c;
    
  }

  // TODO: calculate the mean
    rmse=rmse/estimations.size();

  // TODO: calculate the squared root
  rmse=rmse.array().sqrt();

  // return the result
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
   MatrixXd Hj=MatrixXd(3,4);
  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // TODO: YOUR CODE HERE 

  // check division by zero
  float den=px*px+py*py;
  if (den==0) {
      //std::cout<<"division by Zero"<<std::endl;
      return (Hj);
  }
  
  
  // compute the Jacobian matrix
  Hj<< px/(sqrt(den)),py/(sqrt(den)),0,0,
       -py/den, px/den,0,0,
       py*(vx*py-vy*px)/(den*sqrt(den)),py*(vy*px-vx*py)/(den*sqrt(den)),px/sqrt(den),py/sqrt(den);

  return Hj;
}
double Tools::NormalizeAngle(double angle){
	const double N_PI = 3.14159265358979323846;
    double a = fmod(angle + N_PI, 2 * N_PI);
    return a >= 0 ? (a - N_PI) : (a + N_PI);

	
}

VectorXd Tools::Cartesian2Polar(const VectorXd& x_cartesian){
	
  VectorXd x_polar(3);
  x_polar<<0,0,0;  
	
  float px = x_cartesian(0);
  float py = x_cartesian(1);
  float vx = x_cartesian(2);
  float vy = x_cartesian(3);
  
  float rho=sqrt(px*px+py*py);
  if (rho==0) {
      //std::cout<<"division by Zero"<<std::endl;
      return (x_polar);
  }
  
  double theta=atan2(py/px);
  double rho_dot=(px*vx+py*vy)/(rho);
  
  
  
  x_polar << rho,theta, rho_dot; 
  return x_polar;	
	
	
	
	
}