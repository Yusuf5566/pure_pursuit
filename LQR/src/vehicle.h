//
// Created by yuong on 23-9-5.
//

#ifndef PURE_VEHICLE_H
#define PURE_VEHICLE_H
#include <cmath>
#include <Eigen/Dense>

struct Limit
{
    double v_limit = 10;
    double acc_limit = 1;
    double delta_limit = M_PI_4;  // front wheel turning angle limit
};

struct Vehicle
{
    Eigen::MatrixXd state;
    double v;
    double acc;
    double delta;  // front wheel turning angle
    double L;      // wheel base
};

#endif  // PURE_VEHICLE_H
