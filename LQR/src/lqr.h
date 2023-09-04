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
    LqrController(double dt, double L, double v_desired,std::vector<Vector3d> path);
    void SolveLQRProblem(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B, const Eigen::MatrixXd &Q,
        const Eigen::MatrixXd &R, const double tolerance, const uint max_num_iteration, Eigen::MatrixXd *ptr_K);

    void SolveLQRProblem(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B, const Eigen::MatrixXd &Q,
        const Eigen::MatrixXd &R, const Eigen::MatrixXd &M, const double tolerance, const uint max_num_iteration,
        Eigen::MatrixXd *ptr_K);
    // 通过运动学来进行计算
    void Kinematic();
    // 通过动力学来计算
    void Dynamic();
    // 计算参考点
    MatrixXd reference(const MatrixXd &state);
    // 更新车辆状态
    MatrixXd updateState(const Vector3d &state, double& v, double delta, double dt,MatrixXd &control);
    // LQR 控制器计算
    MatrixXd computeControl(const MatrixXd &state, const MatrixXd &reference);

private:
    Matrix<double, 3, 3> A;  // 系统动态矩阵
    Matrix<double, 3, 2> B;  // 控制矩阵
    Matrix<double, 3, 3> Q;  // LQR权重矩阵 (状态误差)
    Matrix<double, 2, 2> R;  // LQR权重矩阵 (控制输入)
    Matrix3d P;
    Matrix<double, 2, 3> K;
    std::vector<Vector3d> path_;
    double lookAheadDistance = 2;
    double v_;
    double L_;                // 车轮间的距离
    double t_;

};

#endif  // LQR_LQR_H
