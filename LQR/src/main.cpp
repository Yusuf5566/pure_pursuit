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

    Vehicle vehicle;
    vehicle.v = 3.0;
    vehicle.L = 2.5;
    vehicle.delta = 0.0;
    Matrix<double, 3, 1> state;
    state << 0, 0.5, 0.1;  // init state
    vehicle.state = state;

    LqrController controller(path);
    controller.Kinematic(vehicle);  // init kinematic
    Matrix<double, 3, 1> reference;

    std::vector<Matrix<double, 3, 1>> state_container;
    double epsilon = 1.0e-6;
    for (int i = 0; i < N; ++i)
    {
        state_container.push_back(vehicle.state);
        // target site
        reference = controller.reference(vehicle.state);
        if (reference(0) - vehicle.state(0) < epsilon)
        {
            std::cout<<"reached the end!"<<std::endl;
            vehicle.v = 0;
            // 此处为了上面画图的时候数据的统一，所以break注释掉了，不需要画图的话就打开
//            break;
        }
        Matrix<double, 2, 1> control_input = controller.computeControl(vehicle.state, reference);
        MatrixXd input = control_input;
        controller.updateState(vehicle, input);

    }

    PlotFigure(path, state_container);

    return 0;
}
