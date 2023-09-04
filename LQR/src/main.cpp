#include <cmath>
#include "lqr.h"
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;
#define N 300

void PlotFigure(std::vector<Matrix<double, 3, 1>> path, std::vector<Matrix<double, 3, 1>> state_container)
{
    // plot map
    std::vector<double> x(N), y(N), z(N), w(N);
    for (int i = 0; i < N; ++i)
    {
        x.at(i) = path[i](0);
        y.at(i) = path[i](1);
        z.at(i) = state_container[i](0);
        w.at(i) = state_container[i](1);
    }

    // Set the size of output image to 1200x780 pixels
    plt::figure_size(1200, 780);
    // Plot line from given x and y data. Color is selected automatically.
    plt::plot(x, y);
    // Plot a red dashed line from given x and y data.
    plt::plot(z, w, "r--");
    // Set x-axis to interval [0,1000000]
    plt::xlim(0, N / 10);
    plt::ylim(-5, 5);
    // Add graph title
    plt::title("LQR figure");
    // Save the image (file format is determined by the extension)
    plt::save("./basic.svg");
}

int main()
{
    // 设置路径
    double count = 0.1;
    std::vector<Matrix<double, 3, 1>> path(N);
    for (double i = 0; i < N; ++i)
    {
        path[i](0) = count;
        path[i](1) = 0.1 * sin(count);
        path[i](2) = 0.1 * cos(count);
        count = count + 0.1;
    }
    double v = 2.0;        // 初始速度
    double dt = 0.4;       // 步长
    double L = 2.5;        // 轴距
    double delta = 0.0;    // 前轮转角
    Matrix<double, 3, 1> state;
    state << 0, 0.5, 0.1;  // init state

    LqrController controller(L, path);
    controller.Kinematic(v, state(2), dt, delta);  // init kinematic

    Matrix<double, 3, 1> reference;

    std::vector<Matrix<double, 3, 1>> state_container;
    double epsilon = 1.0e-6;
    for (int i = 0; i < N; ++i)
    {
        if (path.at(i)(0) - state(0) < epsilon)
        {
            v = 0;
            //            break;
        }
        state_container.push_back(state);
        // 计算目标位置
        reference = controller.reference(state);
        Matrix<double, 2, 1> control_input = controller.computeControl(state, reference);
        // 这里输出的控制量是增量，因此前轮转角delta有个累加的过程
        MatrixXd input = control_input;
        state = controller.updateState(state, v, delta, dt, input);
        delta += control_input(1);
        std::cout << "Step " << i << ": x=" << state(0) << " y=" << state(1) << " phi=" << state(2) << std::endl;
    }

    PlotFigure(path, state_container);

    return 0;
}