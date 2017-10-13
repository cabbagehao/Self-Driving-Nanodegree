#include "kalman_filter.h"
#include "tools.h"
#include <iostream>
using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;
KalmanFilter::KalmanFilter() {
  P_ = MatrixXd(4, 4); 
  P_ << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1000, 0,
        0, 0, 0, 1000;

  I = MatrixXd::Identity(4, 4);        
}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  /*
    This function never used.
  */
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;


}

void KalmanFilter::Predict() {
  /**
    * Predict the state
  */
  // u = VectorXd(4);
  // u << 0, 0, 0, 0;
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}


void KalmanFilter::Update(const VectorXd &z) {
  /**
    * Update the state by using Kalman Filter equations
  */   
  VectorXd y = z - H_ * x_;
  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();

  x_ = x_ + K * y;
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
    Update the state by using Extended Kalman Filter equations
  */
  double px = x_[0];
  double py = x_[1];
  double vx = x_[2];
  double vy = x_[3];
  double rho = sqrt(px*px + py*py);
  double phi = atan2(py, px);       // atan(y/x) is different from atan2(y,x) !!
  double rhodot = (px*vx + py*vy) / rho;  

  VectorXd z_pred = VectorXd(3);
  z_pred << rho, phi, rhodot;
  VectorXd y = z - z_pred;
  #define _USE_MATH_DEFINES
  while(y(1) < -M_PI)
  {
    cout << "y(1) + M_PI" << y(1) << endl;
    y(1) += 2 * M_PI;
  }
  while(y(1) > M_PI)
  {
    cout << "y(1) - M_PI" << y(1) << endl;
    y(1) -= 2 * M_PI;
  }

  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();
  x_ = x_ + K * y;
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateF(const double &dt)
{
  F_ = MatrixXd(4, 4); 
  F_ << 1, 0, dt, 0,
        0, 1, 0, dt,
        0, 0, 1, 0,
        0, 0, 0, 1;   
}

void KalmanFilter::UpdateQ(const double &dt)
{
  Q_ = MatrixXd(4, 4);
  double dt_2 = dt * dt;
  double dt_3 = dt_2 * dt;
  double dt_4 = dt_3 * dt;   

  int noise_ax = 9;
  int noise_ay = 9;  
  // Q = G * Qv * Gt;
  Q_ << dt_4 * 1/4 * noise_ax,         0,    dt_3 * 1/2 * noise_ax,     0,
           0,         dt_4 * noise_ay * 1/4,    0,      dt_3 * noise_ay* 1/2, 
           dt_3 * 1/2 * noise_ax,         0,    dt_2 * noise_ax,            0,
           0, dt_3 * 1/2 * noise_ay,            0,           dt_2 * noise_ay;  
}          