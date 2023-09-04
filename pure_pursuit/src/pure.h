//
// Created by yuong on 23-8-23.
//

#ifndef PURE_PURE_H
#define PURE_PURE_H
#include <algorithm>
#include <cmath>
#include <iostream>

struct Point
{
    double x;  // x坐标
    double y;  // y坐标
};

struct Pose
{
    Point point;
    double heading;  // 航向角
};

struct ControlCommand
{
    double delta;
    Point point;
};

struct Vehicle
{
    double wheelbase;         // 轴距
    double maxSteeringAngle;  // 最大转角限制
};

class PurePursuitController
{
public:
    PurePursuitController(Vehicle vehicle, std::vector<Point> path);
    Pose UpdatePose(const Pose& currentPose, double delta, double v, double dt);
    ControlCommand calculateControl(const Pose& currentPos);
    double ld(double kv, double v, double l0)
    {
        lookAheadDistance = kv * v + l0;
        return lookAheadDistance;
    }

private:
    Vehicle vehicle_;
    double lookAheadDistance;
    std::vector<Point> path_;
};

#endif  // PURE_PURE_H
