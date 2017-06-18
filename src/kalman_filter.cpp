#include "kalman_filter.h"

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
    // Predict New State
    float x1 = x_[0];
    float x2 = x_[1];
    float x3 = x_[2];
    float x4 = x_[3];
    
    x_ = F_*x_;
    x1 = x_[0];
    x2 = x_[1];
    x3 = x_[2];
    x4 = x_[3];
    
    // Update the State Convariance
    MatrixXd Ft = F_.transpose();
    P_ = F_*P_*Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
    // Calculate the difference betweent he prediction and the measurment values
    
    VectorXd z_pred = H_ * x_;
    VectorXd y = z - z_pred;
    
    UpdateCalcuations(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
    
    // Calculate the difference betweent he prediction and the measurment values
    float px = x_[0];
    float py = x_[1];
    float vx = x_[2];
    float vy = x_[3];
    
    float c1 = sqrtf(px*px + py*py);
    // Get Normalised Angle
    float c2 = atan2(py,px);
    c2 = tools.NormaliseAngle(c2, z[1]);
    
    VectorXd z_pred = VectorXd(3);
    z_pred << c1, c2, (px*vx + py*vy)/c1;
    VectorXd y = z - z_pred;
    
    UpdateCalcuations(y);
}

void KalmanFilter::UpdateCalcuations(const VectorXd &y)
{
    // Calculate all the random matricies
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd PHt = P_ * Ht;
    MatrixXd K = PHt * Si;
    
    x_ = x_ + K*y;
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;
}
