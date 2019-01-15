#include "KalmanFilter.h"

KalmanFilter::KalmanFilter()
{
    angle = 0.0;
    bias = 0.0;
    P[0][0] = 0.0;
    P[0][1] = 0.0;
    P[1][0] = 0.0;
    P[1][1] = 0.0;
};

void KalmanFilter::setAngle(double newAngle)
{
    angle = newAngle;
};

double KalmanFilter::calcAngle(double newAngle, double newRate, double dt)
{

    // variances
    double Q_angle = 0.001;
    double Q_bias = 0.003;
    double R_measure = 0.03;

    // step 1
    double rate = newRate - bias;
    angle += dt * rate;

    // step 2
    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    // step 3
    double y = newAngle - angle;

    // step 4
    double S = P[0][0] + R_measure;

    // step 5
    double K[2];
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    // step 6
    angle += K[0] * y;
    bias += K[1] * y;

    // step 7
    double P00_temp = P[0][0];
    double P01_temp = P[0][1];
    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;

    return angle;
};