#include "lqr.h"

LqrController::LqrController(double dt, double L, double v_desired, std::vector<Vector3d> path)
{
    this->L_ = L;
    this->path_ = path;
    this->t_ = dt;
    this->v_ = v_desired;
}

void LqrController::Kinematic()
{
    double delta = 0.0;
    A << 1, 0, -t_ * v_ * sin(delta), 0, 1, t_ * v_ * cos(delta), 0, 0, 1;

    B << t_ * cos(delta), 0, t_ * sin(delta), 0, t_ * (tan(delta) / L_), t_ * (v_ / (L_ * pow(cos(delta), 2)));

    Q << 100, 0, 0, 0, 100, 0, 0, 0, 100;

    R << 10, 0, 0, 10;
}

void LqrController::Dynamic() {}

// 计算参考点
MatrixXd LqrController::reference(const Eigen::MatrixXd &state)
{
    double closestDistance = std::numeric_limits<double>::max();
    int closestIndex = 0;

    for (int i = 0; i < path_.size(); ++i)
    {
        double distance = std::hypot(path_[i].x() - state(0), path_[i].y() - state(1));
        if (distance < closestDistance)
        {
            closestDistance = distance;
            closestIndex = i;
        }
    }
    MatrixXd reference;
    reference = path_[closestIndex];

    if (path_.back()(0) - state(0) < lookAheadDistance) { reference = path_.back(); }
    else
    {
        for (size_t i = closestIndex + 1; i < path_.size() - 1; i++)
        {
            //              double segmentLength = std::hypot(path_[i+1].x - path_[i].x, path_[i+1].y - path_[i].y);
            double remainingDistance = std::hypot(path_[i](0) - state(0), path_[i](1) - state(1));

            if (remainingDistance > lookAheadDistance)
            {
                double ratio = lookAheadDistance / remainingDistance;
                reference(0) = path_[i](0) + ratio * (path_[i + 1](0) - path_[i](0));
                reference(1) = path_[i](1) + ratio * (path_[i + 1](1) - path_[i](1));
                reference(2) = path_[i](2);
                break;
            }
        }
    }

    return reference;
}

MatrixXd LqrController::computeControl(const MatrixXd &state, const MatrixXd &reference)
{
    // 计算误差
    MatrixXd error = state - reference;
    //    std::cout<<"error:"<<error<<std::endl;

    // LQR 控制器增益矩阵
    MatrixXd *K;
    SolveLQRProblem(A, B, Q, R, error(1), 10, K);
    // 计算控制输入
    MatrixXd control = -*K * error;

    return control;
}

// 这里的delta是所有控制量的累加和,delta有正负，所以外面应该有一个记录delta的变量
MatrixXd LqrController::updateState(const Vector3d &state, double &v, double delta, double dt, MatrixXd &control)
{

    delta = std::clamp(delta, -M_PI_4, M_PI_4);   // 前轮转角最大为pi/4
    v = std::min(v, 5.0);   // 限制最大速度为5m/s

    Matrix<double, 3, 1> newState;
    newState(0) = state(0) + v * cos(state(2)) * dt + (1 / 2) * control(0) * dt * dt * cos(state(2));
    newState(1) = state(1) + v * sin(state(2)) * dt + (1 / 2) * control(0) * dt * dt * sin(state(2));
    newState(2) = (tan(delta) / L_) * dt + (v * dt * control(1)) / L_;
    v = control(0) * dt;

    return newState;
}

// tolerance表示迭代误差 ， max_num_iteration迭代步长N ， ptr_K 增益矩阵K
void LqrController::SolveLQRProblem(const MatrixXd &A, const MatrixXd &B, const MatrixXd &Q, const MatrixXd &R,
    const double tolerance, const uint max_num_iteration, MatrixXd *ptr_K)
{
    // create M as zero matrix of the right size:
    // M.rows() == Q.rows() && M.cols() == R.cols()
    MatrixXd M = MatrixXd::Zero(Q.rows(), R.cols());
    SolveLQRProblem(A, B, Q, R, M, tolerance, max_num_iteration, ptr_K);
}

void LqrController::SolveLQRProblem(const MatrixXd &A, const MatrixXd &B, const MatrixXd &Q, const MatrixXd &R,
    const MatrixXd &M, const double tolerance, const uint max_num_iteration, MatrixXd *ptr_K)
{
    if (A.rows() != A.cols() || B.rows() != A.rows() || Q.rows() != Q.cols() || Q.rows() != A.rows() ||
        R.rows() != R.cols() || R.rows() != B.cols() || M.rows() != Q.rows() || M.cols() != R.cols())
    {
        return;
    }

    MatrixXd AT = A.transpose();
    MatrixXd BT = B.transpose();
    MatrixXd MT = M.transpose();

    // Solves a discrete-time Algebraic Riccati equation (DARE)
    // Calculate Matrix Difference Riccati Equation, initialize P and Q
    MatrixXd P = Q;
    uint num_iteration = 0;
    double diff = std::numeric_limits<double>::max();
    while (num_iteration++ < max_num_iteration && diff > tolerance)
    {
        MatrixXd P_next = AT * P * A - (AT * P * B + M) * (R + BT * P * B).inverse() * (BT * P * A + MT) + Q;
        // check the difference between P and P_next
        diff = fabs((P_next - P).maxCoeff());
        P = P_next;
        //        std::cout<<"p="<<P<<std::endl;
    }

//    if (num_iteration >= max_num_iteration)
//    {
//        std::cout << "LQR solver cannot converge to a solution,last consecutive result diff is:" << diff << std::endl;
//    }
//    else
//    {
//        std::cout << "LQR solver converged at iteration:" << num_iteration << "max consecutive result diff.:" << diff
//                  << std::endl;
//    }
    // transpose()矩阵的转置T，inverse()逆矩阵
    *ptr_K = (R + BT * P * B).inverse() * (BT * P * A + MT);
}