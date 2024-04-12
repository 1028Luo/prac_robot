#include "simple_LQR.hpp"

void LQR::initLQR(){


    u_ref << 1, 1;


    // linearization on v = 0, yaw = 0

    A << 0, 0, 0,
         0, 0, 0,
         0, 0, 0;

    B << 1, 0,
         0, 1,
         0, 0;

    Q << 1, 0, 0,
         0, 1, 0,
         0, 0, 1;

    R << 1, 0,
         0, 1;

    // solve the Riccati equation and get K
    Eigen::Matrix<double, 5, 5> S;
    S = A.transpose() * Q * A + R;
    K = (R.inverse() * B.transpose() * S).transpose();

}

ControlInput LQR::generateControlInput(State currState, State desiredState){


    Eigen::Vector3d u;
    Eigen::Vector3d x;
    Eigen::Vector3d x_ref; // Reference state [x_ref, y_ref, theta_ref]
    
    x << currState.x, currState.y, currState.yaw;
    x_ref << desiredState.x, desiredState.y, desiredState.yaw;

    u = -K * (x - x_ref) + u_ref;
    
    ControlInput input(u[0], u[1]);
    return input;
}
