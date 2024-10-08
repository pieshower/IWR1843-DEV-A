#include <iostream>

#include "../include/kalmanFilter.h"

#define PI 3.14159265

using Eigen::MatrixXd;
using Eigen::VectorXd;

kalmanFilter::kalmanFilter() {
    initKalmanVariables();
}

kalmanFilter::kalmanFilter(const VectorXd &X_in) {
    initKalmanVariables();
    init(X_in);
}

void kalmanFilter::initKalmanVariables() {
    X << 1, 1, 1, 0, 0, 0;

    P << 1, 0, 0, 0, 0, 0, 
         0, 1, 0, 0, 0, 0, 
         0, 0, 1, 0, 0, 0, 
         0, 0, 0, 1, 0, 0, 
         0, 0, 0, 0, 1, 0, 
         0, 0, 0, 0, 0, 1;

    F << 1, 0, 1, 0, 0, 0, 
         0, 1, 0, 1, 0, 0, 
         0, 0, 1, 0, 1, 0, 
         0, 0, 0, 1, 0, 1, 
         0, 0, 0, 0, 1, 0, 
         0, 0, 0, 0, 0, 1;

    Q << 0.1, 0, 0, 0, 0, 0, 
         0, 0.1, 0, 0, 0, 0, 
         0, 0, 0.1, 0, 0, 0, 
         0, 0, 0, 0.1, 0, 0, 
         0, 0, 0, 0, 0.1, 0, 
         0, 0, 0, 0, 0, 0.1;

    H << 1, 0, 0, 0, 0, 0, 
         0, 1, 0, 0, 0, 0, 
         0, 0, 1, 0, 0, 0,
         0, 0, 0, 1, 0, 0;

    R << 0.1, 0, 0, 0, 
         0, 0.1, 0, 0, 
         0, 0, 0.1, 0,
         0, 0, 0, 0.1;
}

void kalmanFilter::init(const VectorXd &X_in) {
    X = X_in;
}

void kalmanFilter::predict() {
    X = F * X;
    MatrixXd F_t = F.transpose();
    P = F * P * F_t + Q;
    std::cout << "predict finish" << std::endl;
}

void kalmanFilter::update(const VectorXd &z) {
    float px = X[0];
    float py = X[1];
    float pz = X[2];
    float vx = X[3];
    float vy = X[4];
    float vz = X[5];

    // Calculate rho, theta, and phi as the predicted measurement
    float rho = sqrt(px * px + py * py + pz * pz);
    float theta = atan2(py, px);
    float phi = acos(pz / rho);

    if (rho < 0.0001) {
        rho = 0.0001;
    }

    // Predicted radial velocity (rho_dot_p)
    float rho_dot_p = (px * vx + py * vy + pz * vz) / rho;

    // Adjusted predicted measurement vector to match MEASR_DIM
    VectorXd z_pred = VectorXd(MEASR_DIM);
    z_pred[0] = rho;
    z_pred[1] = theta;
    z_pred[2] = phi;
    z_pred[3] = rho_dot_p;

    // Measurement residual (difference between actual and predicted)
    VectorXd y = z - z_pred;

    // Normalize angles to ensure they are within [-PI, PI]
    if (y(1) > PI) {
        y(1) -= 2 * PI;
    }
    else if (y(1) < -PI) {
        y(1) += 2 * PI;
    }

    if (y(2) > PI) {
        y(2) -= 2 * PI;
    }
    else if (y(2) < -PI) {
        y(2) += 2 * PI;
    }

    // Kalman filter update step
    MatrixXd H_t = H.transpose();
    MatrixXd PH_t = P * H_t;

    // Measurement covariance S
    MatrixXd S = H * PH_t + R;

    // Kalman gain K
    MatrixXd K = PH_t * S.inverse();

    // Update state estimate
    X = X + K * y;

    // Update the state covariance matrix
    long x_size = X.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P = (I - K * H) * P;

    std::cout << "update finished" << std::endl;
}