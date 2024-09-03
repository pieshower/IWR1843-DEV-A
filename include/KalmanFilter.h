#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include <Eigen/Dense>

#include "../include/mmWaveRadar.h"

class kalmanFilter /* :  public mmWaveRadar */ {
private:

    double A;
    double B;
    double Q;
    double R;
    double P;
    double x_hat;
    double x_hat_new;
    double v_hat;
    double dt;

     kalmanFilter() {}
    ~kalmanFilter() { delete this; } 

public:

};

#endif