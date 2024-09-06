
#include "../include/KalmanFilter.h"

#define PI 3.14159265

using Eigen::MatrixXd;
using Eigen::VectorXd;

void kalmanFilter::init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in, MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
    currentState = x_in;
    covariance = P_in;
    transition = F_in;

    measurement = H_in;
    measurement_covariance = R_in;
    covariance_proccessed = Q_in;
}

void kalmanFilter::predict() {
    currentState = transition * currentState;
    MatrixXd transitionTransposed = transition.transpose();
    covariance = transition * covariance * transitionTransposed + covariance_proccessed;
}

void kalmanFilter::update(const VectorXd &z) {
  
    float px = currentState(0);
    float py = currentState(1);
    float pz = currentState(2);
    float vx = currentState(3);
    float vy = currentState(4);
    float vz = currentState(5);
  
    float rho = sqrt(px * px + py * py + pz * pz);
    float theta = atan(py / px);
    float phi = acos(pz / rho);

    if (rho < 0.0001) {
        rho = 0.0001;
    }
    
    float rho_dot_p = (px * vx + py * vy + pz * vz) / rho;

    VectorXd z_pred = VectorXd(4);
    z_pred << rho, theta, phi, rho_dot_p;

    VectorXd y = z - z_pred;
  
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

    MatrixXd measurementTransposed = measurement.transpose();
    MatrixXd PHt = covariance * measurementTransposed;

    MatrixXd S = measurement * PHt + measurement_covariance;
    MatrixXd K = PHt * S.inverse();

    // Update State
    currentState = currentState + (K * y);
    
    // Update covariance matrix
    long x_size = currentState.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);  
    covariance = (I - K * measurement) * covariance;
}







