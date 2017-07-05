#include "kalman_filter.h"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

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
  TODO:
    * predict the state
  */
    x_ = F_ * x_;
    MatrixXd Ft = F_.transpose();
    P_ = F_* P_* Ft + Q_;    
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  
  UpdateCommon(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  float rho = sqrt(x_(0)*x_(0) + x_(1)*x_(1));
  
  float phi = atan2(0.001, 0.001);
  if (x_(1) > 0.001 && x_(0) > 0.001) phi = atan2(x_(1), x_(0));
  
  float rho_dot = 0.0;
  if (fabs(rho) > 0.0001) rho_dot = (x_(0)*x_(2) + x_(1)*x_(3))/rho;
  
  VectorXd z_pred(3);
  z_pred << rho, phi, rho_dot;
  VectorXd y = z - z_pred;
  NormalizeAngle(y(1));
  
  UpdateCommon(y);
}

void KalmanFilter::UpdateCommon(const VectorXd& y)
{
  const MatrixXd PHt = P_ * H_.transpose();
  const MatrixXd S = H_ * PHt + R_;
  const MatrixXd K = PHt * S.inverse();
  
  //new state
  x_ += K * y;
  P_ -= K * H_ * P_;
}

void KalmanFilter::NormalizeAngle(double& phi)
{
  if (sin(phi) < 0.001 && cos(phi) < 0.001){
    phi = atan2(0.001, 0.001);
  } else {
    phi = atan2(sin(phi), cos(phi));
  }
  
  
}
