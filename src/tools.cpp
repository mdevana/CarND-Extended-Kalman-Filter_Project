#include "tools.h"
#include <iostream>
#define _USE_MATH_DEFINES
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
      //std::cout<<"division by Zero"<<endl;
      return (Hj);
  }
  
  
  // compute the Jacobian matrix
  Hj<< px/(sqrt(den)),py/(sqrt(den)),0,0,
       -py/den, px/den,0,0,
       py*(vx*py-vy*px)/(den*sqrt(den)),py*(vy*px-vx*py)/(den*sqrt(den)),px/sqrt(den),py/sqrt(den);

  return Hj;
}
double Tools::NormalizeAngle(double angle){
	
    double a = fmod(angle + M_PI, 2 * M_PI);
    return a >= 0 ? (a - M_PI) : (a + M_PI);

	
}