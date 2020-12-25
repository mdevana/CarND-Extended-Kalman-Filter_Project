#include "tools.h"
#include <iostream>

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
  /*
  if ((estimations.size()==0) || (estimations.size()!=ground_truth.size())){
    cout<<"invalid inputs";
    return(rmse);
  }
  
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
  rmse=rmse.array().sqrt();*/
  
  return (rmse);
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
  

  // TODO: YOUR CODE HERE 
  
  MatrixXd Hj=MatrixXd(3,4);
  
  Hj << 1.0, 0, 0, 0
          0, 1.0, 0, 0,
          0, 0, 1.0, 0;
  
  
  // recover state parameters
  /*float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // check division by zero
  float den=px*px+py*py;
  if (fabs(den)<0.0001) {
      cout<<"dividion by Zero"<<endl;
      return (Hj);
  }
  
  
  // compute the Jacobian matrix
  Hj<< px/(sqrt(den)),py/(sqrt(den)),0,0,
       -py/den, px/den,0,0,
       py*(vx*py-vy*px)/(den*sqrt(den)),py*(vy*px-vx*py)/(den*sqrt(den)),px/sqrt(den),py/sqrt(den);*/
  return Hj;	
}
