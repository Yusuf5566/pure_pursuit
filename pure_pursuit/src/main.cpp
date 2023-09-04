#include "pure.h"

int main()
{
    std::vector<Point> path;
    double x = 0.0;
    double y = 0.0;
    for (size_t i = 0; i < 500; ++i)
    {
        path.push_back({x, y});
        x = x + 0.1;
    }

    Vehicle vehicle{2.5, M_PI_4};  // 轴距为2.5米，最大转角限制为45度
    PurePursuitController controller(vehicle, path);

    Pose currentPose;
    currentPose.point = {0.5, 0.5};
    currentPose.heading = 0;
    double v = 1.0;   // 速度（1 m/s）
    double dt = 0.1;  // 时间步长（0.1秒）
    double kv = 0.1;
    double l0 = 2;
    controller.ld(kv, v, l0);

    for (size_t i = 0; i < path.size(); i++)
    {
        // 计算目标位置
        ControlCommand targetPoint = controller.calculateControl(currentPose);
        // 基于当前位姿信息计算下一步位姿信息
        Pose nextPose = controller.UpdatePose(currentPose, targetPoint.delta, v, dt);
        // 更新当前位姿信息
         currentPose = nextPose;

        // 目标point
        std::cout << "target point: (" << targetPoint.point.x << ", " << targetPoint.point.y
                  << "), front wheel : " << targetPoint.delta << std::endl;
        // 打印下一步位姿信息
        std::cout << "next point: (" << nextPose.point.x << ", " << nextPose.point.y
                  << "), steering radius: " << nextPose.heading << std::endl;
        std::cout << std::endl;
    }
    return 0;
}
