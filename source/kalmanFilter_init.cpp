#include "../include/kalmanFilter_init.h"

VectorXd X_; // current state
MatrixXd P_; // covariance matrix
MatrixXd F_; // transition matrix

MatrixXd Q_; // process covariance matrix
MatrixXd H_; // measurement matrix
MatrixXd R_; // measurement covariance matrix

void initKalmanVariables() {
    X_ = VectorXd(STATE_DIM);
    P_ = MatrixXd(STATE_DIM, STATE_DIM);
    F_ = MatrixXd(STATE_DIM, STATE_DIM);
    Q_ = MatrixXd(STATE_DIM, STATE_DIM);
    H_ = MatrixXd(MEASR_DIM, STATE_DIM);
    R_ = MatrixXd(MEASR_DIM, MEASR_DIM);    
    
    X_ << 1, 1, 1, 0, 0, 0;

    P_ << 1, 0, 0, 0, 0, 0, 
          0, 1, 0, 0, 0, 0, 
          0, 0, 1, 0, 0, 0, 
          0, 0, 0, 1, 0, 0, 
          0, 0, 0, 0, 1, 0, 
          0, 0, 0, 0, 0, 1;

    F_ << 1, 0, 1, 0, 0, 0, 
          0, 1, 0, 1, 0, 0, 
          0, 0, 1, 0, 1, 0, 
          0, 0, 0, 1, 0, 1, 
          0, 0, 0, 0, 1, 0, 
          0, 0, 0, 0, 0, 1;

    Q_ << 0.1, 0, 0, 0, 0, 0, 
          0, 0.1, 0, 0, 0, 0, 
          0, 0, 0.1, 0, 0, 0, 
          0, 0, 0, 0.1, 0, 0, 
          0, 0, 0, 0, 0.1, 0, 
          0, 0, 0, 0, 0, 0.1;

    H_ << 1, 0, 0, 0, 0, 0, 
          0, 1, 0, 0, 0, 0, 
          0, 0, 1, 0, 0, 0,
          0, 0, 0, 1, 0, 0;

    R_ << 0.1, 0, 0, 0, 
          0, 0.1, 0, 0, 
          0, 0, 0.1, 0,
          0, 0, 0, 0.1;
}