
#include "../include/kalmanFilter.h"

#define PI 3.14159265

using Eigen::MatrixXd;
using Eigen::VectorXd;

void kalmanFilter::init(VectorXd X_in, MatrixXd &P_in, MatrixXd &F_in, MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
    X = X_in;
    P = P_in;
    F = F_in;
    H = H_in;
    R = R_in;
    Q = Q_in;
}

void kalmanFilter::predict() {
    X = F * X;
    MatrixXd F_t = F.transpose();
    P = F * P * F_t + Q;
}

void kalmanFilter::update(const VectorXd &z) {
    float px = X[0];
    float py = X[1];
    float pz = X[2];
    float vx = X[3];
    float vy = X[4];
    float vz = X[5];

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

    if (y[1] > PI) {
        y[1] -= 2 * PI;
    }
    else if (y[1] < -PI) {
        y[1] += 2 * PI;
    }

    if (y[2] > PI) {
        y[2] -= 2 * PI;
    }
    else if (y[2] < -PI) {
        y[2] += 2 * PI;
    }

    MatrixXd H_t = H.transpose();
    MatrixXd PH_t = P * H_t;

    MatrixXd S = H * PH_t + P;
    MatrixXd K = PH_t * S.inverse();

    // Update State
    X = X + (K * y);

    // Update covariance matrix
    long x_size = X.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P = (I - K * H) * P;
}







