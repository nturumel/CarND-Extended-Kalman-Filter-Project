#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

<<<<<<< HEAD

KalmanFilter::KalmanFilter() = default;

KalmanFilter::~KalmanFilter() = default;

MatrixXd KalmanFilter::I_ = MatrixXd::Identity(4, 4);
Tools KalmanFilter::tools_ = Tools();

void KalmanFilter::Init(VectorXd &x_in,       MatrixXd &P_in,       MatrixXd &F_in, MatrixXd &H_in,
                        MatrixXd &R_lidar_in, MatrixXd &R_radar_in, MatrixXd &Q_in) {
  x_       = x_in;
  P_       = P_in;
  F_       = F_in;
  H_       = H_in;
  R_lidar_ = R_lidar_in;
  R_radar_ = R_radar_in;
  Q_       = Q_in;
}

=======
/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */
>>>>>>> adf840f2e2ebfa8626158722f4db6a7ea43274dd
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
<<<<<<< HEAD
}

void KalmanFilter::Predict() {
  x_ = F_ * x_; // u is zero vector; omitted for optimization purposes
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  UpdateCommon(z, H_, R_lidar_, false);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  MatrixXd Hj = tools_.CalculateJacobian(x_);

  UpdateCommon(z, Hj, R_radar_, true);
}

void KalmanFilter::UpdateCommon(const VectorXd &z, const MatrixXd &H, const MatrixXd &R, bool is_ekf) {
  VectorXd y;

  if (is_ekf) {
    // using equations for extended kalman filter

    // convert radar measurements from cartesian coordinates (x, y, vx, vy) to polar (rho, phi, rho_dot).
    VectorXd x_polar = ConvertToPolar(x_);
    y = z - x_polar;

    // normalize the angle between -pi to pi
    while(y(1) > M_PI){
      y(1) -= 2 * M_PI;
    }

    while(y(1) < -M_PI){
      y(1) += 2 * M_PI;
    }
  } else {
    // using equations for kalman filter
    y = z - H * x_;
  }

  MatrixXd Ht = H.transpose();
  MatrixXd S = H * P_ * Ht + R;
  MatrixXd Si = S.inverse();
  MatrixXd K =  P_ * Ht * Si;

  // new state
  x_ = x_ + (K * y);
  P_ = (I_ - K * H) * P_;
}
=======
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

>>>>>>> adf840f2e2ebfa8626158722f4db6a7ea43274dd
