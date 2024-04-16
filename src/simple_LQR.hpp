#include <iostream>
#include <Eigen/Eigen>

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

    ControlInput generateControlInput(State currState, State desiredState, double dt);


private:

    Eigen::Matrix3d A;
    Eigen::Matrix<double, 3, 2> B;

    Eigen::Matrix3d Q;
    Eigen::Matrix2d R;

    std::vector<Eigen::MatrixXd> K;

    Eigen::Vector2d u_ref; 

    uint8_t N = 50; // number of iteration
    std::vector<Eigen::MatrixXd> P;
};