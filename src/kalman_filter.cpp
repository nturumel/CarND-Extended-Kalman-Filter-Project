#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */
Eigen::VectorXd ConvertToPolar(const VectorXd &x)
{
  VectorXd result(3);
  const double & px=x(0);
  const double & py=x(1);
  const double & vx=x(2);
  const double & vy=x(3);
  
  double rho,phi,rhoDot;
  rho=sqrt(px*px+py*py);
  phi=atan2(py,px);
  
  if(rho<0.0000001)
    rho=0.0000001;
  
  rhoDot=(px*vx+py*vy)/rho;
  
  result<<rho,phi,rhoDot;
  return result; 
}


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

void KalmanFilter::Predict() 
{
  /**
   * TODO: predict the state
   **/
  
  x_=F_*x_;
  MatrixXd Ft=F_.transpose();
  P_=F_*P_*Ft+Q_;
  
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  MatrixXd I = MatrixXd::Identity(2, 2);
  VectorXd y = z-H_*x_;
  MatrixXd Ht=H_.transpose();
  MatrixXd S=H_*P_*Ht+R_;
  MatrixXd Sinv=S.inverse();
  MatrixXd k=P_*Ht*Sinv;
  
  x_=(x_+k*y);
  P_=(I-k*H_)*P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  VectorXd xRho=ConvertToPolar(x_);
  MatrixXd I = MatrixXd::Identity(2, 2);
  VectorXd y = z-H_*xRho;
  if(y(1)> M_PI)
    y(1)=y(1)-2*M_PI;
  
  if(y(1)< M_PI)
    y(1)=y(1)+2*M_PI;
    
  
  MatrixXd Ht=H_.transpose();
  MatrixXd S=H_*P_*Ht+R_;
  MatrixXd Sinv=S.inverse();
  MatrixXd k=P_*Ht*Sinv;
  
  x_=(x_+k*y);
  P_=(I-k*H_)*P_;
  
}

