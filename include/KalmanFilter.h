#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include <eigen3/Eigen/Dense>

#include "../include/mmWaveRadar.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class kalmanFilter {
private:                         
    VectorXd currentState;  // x 
    MatrixXd covariance;    // P
    MatrixXd transition;    // F

    MatrixXd covariance_proccessed;   // Q
    MatrixXd measurement;             // H
    MatrixXd measurement_covariance;  // R

public:
     kalmanFilter() {};
    ~kalmanFilter() { delete this; }

    void init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in, MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in);
    void predict();
    void update(const Eigen::VectorXd &z_);
};

#endif