#ifndef KALMANFILTER_INIT_H
#define KALMANFILTER_INIT_H

#include <eigen3/Eigen/Dense>

#define STATE_DIM 6
#define MEASR_DIM 3

using Eigen::MatrixXd;
using Eigen::VectorXd;

inline VectorXd X_; // current state
inline MatrixXd P_; // covariance matrix
inline MatrixXd F_; // transition matrix

inline MatrixXd Q_; // process covariance matrix
inline MatrixXd H_; // measurement matrix
inline MatrixXd R_; // measurement covariance matrix

inline void initKalmanVariables() {
    X_ = VectorXd(STATE_DIM);
    P_ = MatrixXd(STATE_DIM, STATE_DIM);
    F_ = MatrixXd(STATE_DIM, STATE_DIM);
    Q_ = MatrixXd(STATE_DIM, STATE_DIM);
    H_ = MatrixXd(STATE_DIM, MEASR_DIM);
    R_ = MatrixXd(MEASR_DIM, MEASR_DIM);    
    
    X_ << 1, 1, 0, 0, 0, 0;

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
          0, 0, 1, 0, 0, 0;

    R_ << 0.1, 0, 0, 
          0, 0.1, 0, 
          0, 0, 0.1;
}

#endif