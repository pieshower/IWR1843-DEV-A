#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include <eigen3/Eigen/Dense>

#define STATE_DIM 6
#define MEASR_DIM 4

using Eigen::MatrixXd;
using Eigen::VectorXd;

class kalmanFilter {
private:                         
    VectorXd X = VectorXd(STATE_DIM); // current state
    MatrixXd P = MatrixXd(STATE_DIM, STATE_DIM); // covariance matrix
    MatrixXd F = MatrixXd(STATE_DIM, STATE_DIM); // transition matrix
    MatrixXd Q = MatrixXd(STATE_DIM, STATE_DIM); // process covariance matrix
    MatrixXd H = MatrixXd(MEASR_DIM, STATE_DIM); // measurement matrix
    MatrixXd R = MatrixXd(MEASR_DIM, MEASR_DIM); // measurement covariance matrix

    void initKalmanVariables();
    void init(const VectorXd &X_in);

public:
     kalmanFilter();
     kalmanFilter(const VectorXd &X_in);
    ~kalmanFilter() = default;

    void predict();
    void update(const VectorXd &z_);
};

#endif