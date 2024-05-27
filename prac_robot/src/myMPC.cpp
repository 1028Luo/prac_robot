// Written by Jiexing Luo, email:jiexingluo@Outlook.com
//
// This file contains the path planner myController for the project prac_robot
// By default it uses a simple A* as its path planning algorithm 
//
// myController subscribes to three topics:
//      map - to get map information
//      goal_pose - to get any goal pose made in RVIZ
//      amcl_pose - to get current pose, covariance discarded 
// 
// myController publishes the generated path to myController


#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp" //AMCL
#include "nav_msgs/msg/odometry.hpp" //odom
#include "plan_helper.hpp"
#include "simple_LQR.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"


const int predictionHorizon = 10;
const double timeStep = 0.1;
// const int dimX = 3;
// const int dimU = 2;
// const int uNums = predictionHorizon * dimU; // numbers of predicted control inputs
const double targetX = 5.0; 
const double targetY = -2.0;
const double lowerU = -1.0; // lower and upper constraints for control inputs
const double upperU = 1.0;
const double regularizerWeight = 0.3;
const double lookaheadDist_ = 0.05;

template<typename T>
T norminalAngle(T value){
    while(*(double*) &value > M_PI || *(double*) &value < -M_PI){
        if(*(double*) &value > M_PI) value -= 2.0 * T(M_PI);
        else value += 2.0 *  T(M_PI);
    }
    return value;
}

struct MPCProblem {
  double x_init, y_init, yaw_init;
  double x_goal, y_goal;
  
  MPCProblem(double x_init, double y_init, double yaw_init, double x_goal, double y_goal)
      : x_init(x_init), y_init(y_init), yaw_init(yaw_init), x_goal(x_goal), y_goal(y_goal) {}
  
  template<typename T>
  bool operator()(const T* const v, const T* const w, T* residual) const {
    // State variables
    T x = T(x_init);
    T y = T(y_init);
    T yaw = T(yaw_init);
    
    // Control inputs
    T dt = T(timeStep); // Time step
    T v_k = T(v[0]);
    T w_k = T(w[0]);
    
    // MPC iterations
    for (int i = 0; i < predictionHorizon; ++i) {
      // Update state using kinematic model
      x += v_k * cos(yaw) * dt;
      y += v_k * sin(yaw) * dt;
      yaw += norminalAngle( w_k * dt);

    // Compute residual (distance to goal)
    T x_diff = T(x_goal) - x;
    T y_diff = T(y_goal) - y;
    residual[0] = regularizerWeight * x_diff;
    residual[1] = regularizerWeight * y_diff;
    }
    
    return true;
  }
};




class myController : public rclcpp::Node {
public:
    myController() : Node("my_controller") {

        // Subscribe to topic: map
        path_subscription = this->create_subscription<nav_msgs::msg::Path>(
            "path", 10, std::bind(&myController::pathCallback, this, std::placeholders::_1));
        
        // Subscribe to topic: AMCL_pose
        AMCL_pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "amcl_pose", 10, std::bind(&myController::AMCL_Callback, this, std::placeholders::_1));

        // publish to topic: diff_drive_base_controller/cmd_vel_unstamped
        control_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("diff_drive_base_controller/cmd_vel_unstamped", 10);
    }

private:
    void pathCallback(const nav_msgs::msg::Path::SharedPtr path_msg) {

        std::cout << "received path" << std::endl;

        if (path_msg) {
            path_msg_ = *path_msg;
            myLQR.initLQR();
            if (path_msg_.poses.size() > 1){
                flag_path = true;
            } else{
                std::cout << "myController: error: empty path" << std::endl;
            }
            
            //controlLoop();
        } else {
            RCLCPP_WARN(this->get_logger(), "Received null map message");
        }
    }

    void AMCL_Callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr AMCL_pose_msg) {

        std::cout << "received AMCL pose" << std::endl;
        if (AMCL_pose_msg) {

            std::cout << "Current AMCL pose: ";
            std::cout << "X: " << AMCL_pose_msg->pose.pose.position.x << std::endl;
            std::cout << "Y: " << AMCL_pose_msg->pose.pose.position.y << std::endl;
            std::cout << "rot Z: " << AMCL_pose_msg->pose.pose.orientation.z << std::endl;
            std::cout << std::endl;
            //point temp = pose2map(AMCL_pose_msg->pose.pose.position.x, AMCL_pose_msg->pose.pose.position.y);

            currState_AMCL.x = AMCL_pose_msg->pose.pose.position.x;
            currState_AMCL.y = AMCL_pose_msg->pose.pose.position.y;

            tf2::Quaternion quat(AMCL_pose_msg->pose.pose.orientation.x, AMCL_pose_msg->pose.pose.orientation.y, AMCL_pose_msg->pose.pose.orientation.w, AMCL_pose_msg->pose.pose.orientation.z);
            tf2::Matrix3x3 mat(quat);
            double roll, pitch, yaw;
            mat.getRPY(roll, pitch, yaw);
            currState_AMCL.yaw = yaw;
            flag_curr_pose = true; // start ready
            
        } else {
            RCLCPP_WARN(this->get_logger(), "Received null pose message");
        }
    }

    void controlLoop(){

        if (flag_curr_pose && !flag_reached_target && flag_path){

            std::cout << "myController: in control" << std::endl;

            State desiredState;
            std::cout << "path size:" << path_msg_.poses.size() << std::endl;

            std::cout << "Following point :" << path_idx << std::endl;
            
            // track a waypoint in path
            auto s = path_msg_.poses[path_idx];
            desiredState.x = s.pose.position.x;
            desiredState.y = s.pose.position.y;

            // extract yaw from quaternion 
            tf2::Quaternion quat(s.pose.orientation.x, s.pose.orientation.y, s.pose.orientation.z, s.pose.orientation.w);
            tf2::Matrix3x3 mat(quat);
            double roll, pitch, yaw;
            mat.getRPY(roll, pitch, yaw);

            desiredState.yaw = yaw;











            // get control input
            //ControlInput input = myLQR.generateControlInput(currState_AMCL, desiredState, dt_);
            

            
            //cmd_msg.linear.x = input.linear_vel_x;
            //cmd_msg.angular.z = input.angular_vel_z;


            // publish control command
            std::cout << "myController: sending control input" << std::endl;
            std::cout << "Linear X:" << cmd_msg.linear.x << std::endl;
            std::cout << "Angular Z:" << cmd_msg.angular.z << std::endl;

            control_publisher_->publish(cmd_msg);

            // if this way point has been reached, move to next waypoint
             std::cout << "curr error:" << compute_error(currState_AMCL, desiredState) << std::endl;
            if(compute_error(currState_AMCL, desiredState) <= tolerance) {
                
                path_idx = path_idx + 10;
                if (path_idx > path_msg_.poses.size()){
                    path_idx = path_msg_.poses.size()-1;
                }
            }
        }
    }

    double compute_error(State curr, State desired) {
        auto x_squared = (curr.x - desired.x) * (curr.x - desired.x);
        auto y_squared = (curr.y - desired.y) * (curr.y - desired.y);

        return sqrt(x_squared + y_squared);
    }




    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscription;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr AMCL_pose_subscription_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr control_publisher_;



    // stores message from the subscribers
    double dt_ = 0.05;
    bool flag_curr_pose = false;
    bool flag_reached_target = false;
    bool flag_path = false;
    nav_msgs::msg::Path path_msg_;
    geometry_msgs::msg::Twist cmd_msg;
    LQR myLQR;
    State currState_AMCL;
    State currState_odom;

    int path_idx = 0;
    double tolerance = 1.5;
    double vel_linear_x_cap = 1.0;
    double vel_angualr_z_cap = 1.0;
};

int main(int argc, char** argv) {
    std::cout << "myController is running" << std::endl;
    
    std::cout << std::endl;
    rclcpp::init(argc, argv);
    auto node = std::make_shared<myController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}