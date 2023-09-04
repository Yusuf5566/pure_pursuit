//
// Created by yuong on 23-8-25.
//

#ifndef LQR_LQR_H
#define LQR_LQR_H
#include <Eigen/Dense>
#include <cmath>
#include <eigen3/Eigen/Core>
#include <iostream>
#include <vector>

using namespace Eigen;

class LqrController
{
public:
    LqrController( double L,std::vector<Vector3d> path);
    void SolveLQRProblem(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B, const Eigen::MatrixXd &Q,
        const Eigen::MatrixXd &R, const double tolerance, const uint max_num_iteration, Eigen::MatrixXd *ptr_K);

    void SolveLQRProblem(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B, const Eigen::MatrixXd &Q,
        const Eigen::MatrixXd &R, const Eigen::MatrixXd &M, const double tolerance, const uint max_num_iteration,
        Eigen::MatrixXd *ptr_K);
    // kinematic
    void Kinematic(double v , double phi ,double dt ,double delta);
    // dynamic
    void Dynamic();
    // reference
    MatrixXd reference(const MatrixXd &state);
    // Update state
    MatrixXd updateState(const Vector3d &state, double& v, double delta, double dt,MatrixXd &control);
    // LQR control
    MatrixXd computeControl(const MatrixXd &state, const MatrixXd &reference);

private:
    Matrix<double, 3, 3> A;  // state matrix
    Matrix<double, 3, 2> B;  // control matrix
    Matrix<double, 3, 3> Q;  // Q matrix
    Matrix<double, 2, 2> R;  // R matrix
    Matrix3d P;
    Matrix<double, 2, 3> K;
    std::vector<Vector3d> path_;
    double lookAheadDistance = 1;
    double L_;                // wheel base

};

#endif  // LQR_LQR_H
