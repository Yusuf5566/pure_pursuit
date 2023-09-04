#include "lqr.h"
#include <cmath>
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

void PlotFigure(std::vector<Matrix<double, 3, 1>> path,std::vector<Matrix<double, 3, 1>> state_container)
{
    // plot map
    int n = 200;
    std::vector<double> x(n), y(n), z(n), w(n);
    for (int i = 0; i < n; ++i)
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
    plt::xlim(0, 20 );
    plt::ylim(-10, 20 );
    // Add graph title
    plt::title("LQR figure");
    // Save the image (file format is determined by the extension)
    plt::save("./basic.svg");
}

int main()
{
    // 设置路径
    std::vector<Matrix<double, 3, 1>> path(200);
    for (double i = 0; i < 200; ++i)
    {
        static double count = 0.1;
        path[i](0) = count;
        path[i](1) = 0.1 * sin(count);
        path[i](2) = 0.1 * cos(count);
        count = count + 0.1;
    }
    double v = 2.0;
    double dt = 0.3;
    double L = 2.5;
    double delta = 0.0;

    LqrController controller(dt, L, v, path);
    controller.Kinematic();

    Matrix<double, 3, 1> state;
    state << 0, 0.5, 0;  // init state
    Matrix<double, 3, 1> target;

    std::vector<Matrix<double, 3, 1>> state_container;
    for (int i = 0; i < 200; ++i)
    {
        state_container.push_back(state);
        // 计算目标位置
        target = controller.reference(state);
        Matrix<double, 2, 1> control_input = controller.computeControl(state, target);
        delta += control_input(1);
        MatrixXd input = control_input;
        state = controller.updateState(state, v, delta, dt, input);
//        std::cout << "Step " << i << ": x=" << state(0) << " y=" << state(1)
//                  << " phi=" << state(2)  << std::endl;
    }

    PlotFigure(path,state_container);

    return 0;
}