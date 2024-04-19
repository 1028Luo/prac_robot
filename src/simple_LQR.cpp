#include "simple_LQR.hpp"

void LQR::initLQR(){

    // linearization on v = 0, yaw = 0

    A << 1, 0, 0,
         0, 1, 0,
         0, 0, 1;

    // increase Q and decrease R for higher control effort

    Q << 1, 0, 0,
        0, 1, 0,
        0, 0, 0.8;
        
    R << 0.01, 0,
        0, 0.01;




	std::cout << "LQR:: LQR created" << std::endl;

}


Eigen::Matrix<double, 3, 2> getB(double yaw, double dt){
    Eigen::Matrix<double, 3, 2> B; // Input model
    B << std::cos(yaw)*dt, 0,
        std::sin(yaw)*dt, 0,
        0, dt;

    return B;
}

ControlInput LQR::generateControlInput(State currState, State desiredState, double dt){

    Eigen::Vector3d x;
    Eigen::Vector3d x_ref; // Reference state [x_ref, y_ref, theta_ref]
	x << currState.x, currState.y, currState.yaw;
    x_ref << desiredState.x, desiredState.y, desiredState.yaw;
	
    uint8_t N = 50; // number of iteration
    std::vector<Eigen::MatrixXd> P(N+1);
    Eigen::MatrixXd Qf = Q;
    P[N] = Qf;
    B = getB(currState.yaw, dt);

    for (uint8_t i = N; i >= 1; --i) {
    auto Y = R + B.transpose() * P[i] * B;
    // Compute the SVD decomposition of Y
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(Y, Eigen::ComputeThinU | Eigen::ComputeThinV);
    // Compute the pseudo-inverse of Y
    Eigen::MatrixXd Yinv = svd.matrixV() * svd.singularValues().asDiagonal().inverse() * svd.matrixU().transpose();

    P[i-1] = Q + A.transpose() * P[i] * A - (A.transpose() * P[i] * B) * Yinv * (B.transpose() * P[i] * A);
    }

    std::vector<Eigen::MatrixXd> K(N);
    std::vector<Eigen::Vector2d> u(N);
    
    for (uint8_t i = 0; i <= N-1; ++i){
    auto Y = R + B.transpose() * P[i+1] * B;
    // Compute the SVD decomposition of Y
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(Y, Eigen::ComputeThinU | Eigen::ComputeThinV);
    // Compute the pseudo-inverse of Y
    Eigen::MatrixXd Yinv = svd.matrixV() * svd.singularValues().asDiagonal().inverse() * svd.matrixU().transpose();

    K[i] = Yinv * B.transpose() * P[i+1] * A;
    u[i] = -K[i] * (x - x_ref);
    }

    ControlInput u_optimal = {u[N-1](0), u[N-1](1)};

    std::cout << "LQR:: curr state x is: " << currState.x << std::endl;
    std::cout << "LQR:: desired state x is: " << desiredState.x << std::endl;
    std::cout << "LQR:: curr state y is: " << currState.y << std::endl;
    std::cout << "LQR:: desired state y is: " << desiredState.y << std::endl;
    std::cout << "LQR:: curr state yaw is: " << currState.yaw << std::endl;
    std::cout << "LQR:: desired state yaw is: " << desiredState.yaw << std::endl;



    return u_optimal;
}
