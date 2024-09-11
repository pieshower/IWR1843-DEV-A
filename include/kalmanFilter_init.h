#ifndef KALMANFILTER_INIT_H
#define KALMANFILTER_INIT_H

#include <eigen3/Eigen/Dense>

#define STATE_DIM 6
#define MEASR_DIM 3

using Eigen::MatrixXd;
using Eigen::VectorXd;

VectorXd X_ = VectorXd(STATE_DIM); // current state
MatrixXd P_ = MatrixXd(STATE_DIM, STATE_DIM); // covariance matrix
MatrixXd F_ = MatrixXd(STATE_DIM, STATE_DIM); // transition matrix

MatrixXd Q_ = MatrixXd(STATE_DIM, STATE_DIM); // process covariance matrix
MatrixXd H_ = MatrixXd(STATE_DIM, MEASR_DIM); // measurement matrix
MatrixXd R_ = MatrixXd(MEASR_DIM, MEASR_DIM); // measurement covariance matrix

void initKalmanVariables() {
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