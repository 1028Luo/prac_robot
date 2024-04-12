#include <iostream>
#include <eigen3/Eigen/Dense>

struct State {

    double x;
    double y;
    double yaw;

    State(){}
    State(double x_, double y_, double yaw_): x(x_), y(y_), yaw(yaw_){}

};

struct ControlInput {

    double linear_vel_x;
    double angular_vel_z;

    ControlInput(){}
    ControlInput(double linear_vel_x_, double angular_vel_z_): linear_vel_x(linear_vel_x_), angular_vel_z(angular_vel_z_){}

};


class LQR{


public:
    // use default constructor

    void initLQR(); // init

    ControlInput generateControlInput(State currState, State desiredState);


private:

    Eigen::Matrix3d A;
    Eigen::Matrix<double, 3, 2> B;

    Eigen::Matrix3d Q;
    Eigen::Matrix2d R;

    Eigen::Matrix<double, 2, 3> K;

    Eigen::Vector2d u_ref; 

};