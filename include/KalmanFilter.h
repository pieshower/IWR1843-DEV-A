#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include <Eigen/Dense>

#include "../include/mmWaveRadar.h"

class kalmanFilter /* :  public mmWaveRadar */ {
private:

    float matrixA;
    float matrixB;
    float Q;
    float R;
    float P;

    float position_i;
    float position_n;
    float velocity_i;

    const float time_step = 1.0;


public:
     kalmanFilter() {};
     kalmanFilter(float inital_pos, float initial_vel);
    ~kalmanFilter() { delete this; } 

};

#endif