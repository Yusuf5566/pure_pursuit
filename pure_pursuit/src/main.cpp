#include "pure.h"
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;
#define N 300

void PlotFigure(std::vector<Point> path, std::vector<Pose> pose_container)
{
    // plot map
    std::vector<double> x(N), y(N), x1(N), y1(N);
    for (int i = 0; i < N; ++i)
    {
        x.at(i) = path[i].x;
        y.at(i) = path[i].y;
        x1.at(i) = pose_container[i].point.x;
        y1.at(i) = pose_container[i].point.y;
    }

    // Set the size of output image to 1200x780 pixels
    plt::figure_size(1200, 780);
    // Plot line from given x and y data. Color is selected automatically.
    plt::plot(x, y);
    // Plot a red dashed line from given x and y data.
    plt::plot(x1, y1, "r--");
    // Set x-axis to interval [0,1000000]
    plt::xlim(0, N / 10);
    plt::ylim(-5, 5);
    // Add graph title
    plt::title("pp figure");
    // Save the image (file format is determined by the extension)
    plt::save("./basic.svg");
}

int main()
{
    std::vector<Point> path;
    double x = 0.0;
    double y = 0.0;
    for (size_t i = 0; i < N; ++i)
    {
        path.push_back({x, y});
        x = x + 0.1;
//        y = 0.1 * sin(x);
    }

    Vehicle vehicle{2.5, M_PI_4};  // 轴距为2.5米，最大转角限制为45度
    PurePursuitController controller(vehicle, path);

    Pose currentPose;
    currentPose.point = {0.5, 0.5};
    currentPose.heading = 0;
    double v = 1.0;   // velocity（1 m/s）
    double dt = 0.1;  // step（0.1秒）
    double kv = 0.1;
    double l0 = 1;
    controller.ld(kv, v, l0);
    double epsilon = 1.0e-6;
    std::vector<Pose> pose_container;
    for (size_t i = 0; i < path.size(); i++)
    {
        // target pose
        ControlCommand targetPoint = controller.calculateControl(currentPose);
        if (targetPoint.point.x - currentPose.point.x   < epsilon)
        {
            std::cout<<"reached the end!"<<std::endl;
            v = 0;
            // 此处为了上面画图的时候数据的统一，所以break注释掉了，不需要画图的话就打开
            //            break;
        }
        // next pose
        Pose nextPose = controller.UpdatePose(currentPose, targetPoint.delta, v, dt);
        // update pose
         currentPose = nextPose;
         pose_container.push_back(currentPose);
    }

    PlotFigure(path,pose_container);
    return 0;
}
