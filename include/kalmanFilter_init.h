#ifndef KALMANFILTER_INIT_H
#define KALMANFILTER_INIT_H

#include <eigen3/Eigen/Dense>

#define STATE_DIM 6
#define MEASR_DIM 4

using Eigen::MatrixXd;
using Eigen::VectorXd;

extern VectorXd X_; // current state
extern MatrixXd P_; // covariance matrix
extern MatrixXd F_; // transition matrix

extern MatrixXd Q_; // process covariance matrix
extern MatrixXd H_; // measurement matrix
extern MatrixXd R_; // measurement covariance matrix

void initKalmanVariables();

#endif