#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

using namespace std;
Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
  
  // there are 4 vectors here...
  //px,py,vx,vy
  VectorXd result(4);
  result<<0,0,0,0;
  
  if (estimations.size()!=ground_truth.size())
  {
    cout<<"problem with the data"<<endl;
    return result;
  }
  for(int i=0;i<estimations.size();++i)
  {
    VectorXd residual=estimations[i]-ground_truth[i];
    residual=residual.array()*residual.array();
    result+=residual;
  }
  
  result=result/estimations.size();
  result=result.array().sqrt();
  
  return result;
    
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
  MatrixXd Hj(3,4);
  float px=x_state(0);
  float py=x_state(1);
  float vx=x_state(2);
  float vy=x_state(3);
  
  float c1=px*px+py*py;
  float c2=sqrt(c1);
  float c3=c1*c2;
  
  if(fabs(c1)<0.0001)
  {
    std::cout<<"division by zero"<<std::endl;
    return Hj;
  }
  Hj<<(px/c2),(py/c2),0,0
    ,-(py/c1),(px/c1),0,0
    ,py*(vx*py-vy*px)/c3,px*(px*vy-py*vx)/c3,px/c2,py/c2;
  
  return Hj;
}

