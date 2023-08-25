//
// Created by yuong on 23-8-23.
//

#include "pure.h"

PurePursuitController::PurePursuitController(Vehicle vehicle, std::vector<Point> path)
    : vehicle_(vehicle)
    , path_(path)
{
}

Pose PurePursuitController::UpdatePose(const Pose &currentPose, double delta, double v, double dt)
{
    // 计算下一步车辆的位姿信息
    Pose nextPose;

    // 计算前轮转角和速度限制
    double steeringAngle = std::clamp(delta, -vehicle_.maxSteeringAngle, vehicle_.maxSteeringAngle);
    double speed = std::min(v, 5.0);  // 限制最大速度为5 m/s

    // 根据车辆动力学模型，预测下一步车辆的位置
    double radius = vehicle_.wheelbase / std::tan(steeringAngle);
    double angularVelocity = speed / radius;  // omega
    double deltaHeading = angularVelocity * dt;

    nextPose.point.x = currentPose.point.x + speed * std::cos(currentPose.heading) * dt;
    nextPose.point.y = currentPose.point.y + speed * std::sin(currentPose.heading) * dt;
    nextPose.heading = currentPose.heading + deltaHeading;

    return nextPose;
}

ControlCommand PurePursuitController::calculateControl(const Pose &currentPos)
{
    double closestDistance = std::numeric_limits<double>::max();
    int closestIndex = 0;

    // Find the closest point on the path
    for (size_t i = 0; i < path_.size(); i++)
    {
        double distance = std::hypot(path_[i].x - currentPos.point.x, path_[i].y - currentPos.point.y);
        if (distance < closestDistance)
        {
            closestDistance = distance;
            closestIndex = i;
        }
    }

    Point target;
    // Find the point on the path that is look ahead distance away
    //    double epsilon = 1.0e-6;
    if (path_.back().x - currentPos.point.x < lookAheadDistance)
    {
        target.x = path_.back().x;
        target.y = path_.back().y;
    }
    else
    {
        for (size_t i = closestIndex + 1; i < path_.size() - 1; i++)
        {
            //              double segmentLength = std::hypot(path_[i+1].x - path_[i].x, path_[i+1].y - path_[i].y);
            double remainingDistance = std::hypot(path_[i].x - currentPos.point.x, path_[i].y - currentPos.point.y);

            if (remainingDistance > lookAheadDistance)
            {
                double ratio = lookAheadDistance / remainingDistance;
                target.x = path_[i].x + ratio * (path_[i + 1].x - path_[i].x);
                target.y = path_[i].y + ratio * (path_[i + 1].y - path_[i].y);
                break;
            }
        }
    }

    ControlCommand command;
    Point matrix_T{target.x - currentPos.point.x, target.y - currentPos.point.y};
    Point matrix_O{currentPos.point.x + vehicle_.wheelbase * cos(currentPos.heading),
        currentPos.point.y + vehicle_.wheelbase * sin(currentPos.heading)};
    // 向量叉乘计算alpha
    double sin_alpha =
        (matrix_T.x * matrix_O.y - matrix_T.y * matrix_O.x) /
        (sqrt(pow(matrix_T.x, 2) + pow(matrix_T.y, 2)) * (sqrt(pow(matrix_O.x, 2) + pow(matrix_O.y, 2))));
    // 这里的delta要有正负，往左为正，往右为负
    command.delta = -std::clamp(atan((2 * vehicle_.wheelbase * sin_alpha) /
                                     std::hypot(target.x - currentPos.point.x, target.y - currentPos.point.y)),
        -vehicle_.maxSteeringAngle,
        vehicle_.maxSteeringAngle);
    command.point = target;
    std::cout << "ey=" << 2 * sin_alpha << command.delta << std::endl;
    return command;
}