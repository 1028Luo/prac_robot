// Written by Jiexing Luo, email:jiexingluo@Outlook.com
//
// This file contains the path planner myPlanner for the project prac_robot
// By default it uses a simple A* as its path planning algorithm 
//
// myPlanner subscribes to three topics:
//      map - to get map information
//      goal_pose - to get any goal pose made in RVIZ
//      amcl_pose - to get current pose, covariance discarded 
// 
// myPlanner publishes the generated path to myController


#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp" // map
#include "nav_msgs/msg/path.hpp" // map
#include "geometry_msgs/msg/pose_stamped.hpp" // goal_pose
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp" //AMCL
#include "geometry_msgs/msg/quaternion.hpp"
#include "simple_Astar.hpp"
#include "plan_helper.hpp"
#include <string>
#include <opencv2/opencv.hpp>

using namespace cv;

class myPlanner : public rclcpp::Node {
public:
    myPlanner() : Node("my_planner") {


        // Subscribe to topic: map
        map_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "map", 10, std::bind(&myPlanner::mapCallback, this, std::placeholders::_1));

        // Subscribe to topic: goal_pose
        goal_pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "goal_pose", 10, std::bind(&myPlanner::goalCallback, this, std::placeholders::_1));

        // Subscribe to topic: AMCL_pose
        AMCL_pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "amcl_pose", 10, std::bind(&myPlanner::AMCL_Callback, this, std::placeholders::_1));

        path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("path", 10);
    }

private:
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map_msg) {

        std::cout << "received map" << std::endl;

        if (map_msg) {
            
            // access the info field
            std::cout << "Map Info: ";
            std::cout << "Width: " << map_msg->info.width << ", ";
            std::cout << "Height: " << map_msg->info.height << ", ";
            std::cout << "Resolution: " << map_msg->info.resolution << std::endl;

            // stores data in 2d array, initialise all to 0
            std::vector<std::vector<int>> map_temp(map_msg->info.height, std::vector<int>(map_msg->info.width,0));

            for (size_t i =0 ; i < map_msg->info.height; i++){
                for (size_t j = 0; j < map_msg->info.width; j++){
                    auto data_idx = i * map_msg->info.width + j;
                    map_temp[i][j] = map_msg->data[data_idx];
                }
            }
            
            std::cout << "2d occupancy grid made" << std::endl;
            std::cout << std::endl;
            map = map_temp;

            myAstar.initAstar(map);
            flag_map = true; // map ready

            // print map
            //printMap(map);

        } else {
            RCLCPP_WARN(this->get_logger(), "Received null map message");
        }
    }

    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr goal_pose_msg) {

        std::cout << "received goal pose" << std::endl;
        if (goal_pose_msg) {

            std::cout << "Goal pose: ";
            std::cout << "X: " << goal_pose_msg->pose.position.x << std::endl;
            std::cout << "Y: " << goal_pose_msg->pose.position.y << std::endl;
            std::cout << "rot Z: " << goal_pose_msg->pose.orientation.z << std::endl;
            std::cout << std::endl;

            goal_quat.x = goal_pose_msg->pose.orientation.x;
            goal_quat.y = goal_pose_msg->pose.orientation.y;
            goal_quat.w = goal_pose_msg->pose.orientation.w;
            goal_quat.z = goal_pose_msg->pose.orientation.z;

            dest = pose2map(goal_pose_msg->pose.position.x, goal_pose_msg->pose.position.y);

            flag_dest = true;
            publishPath();
        } else {
            RCLCPP_WARN(this->get_logger(), "Received null goal message");
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

            start = pose2map(AMCL_pose_msg->pose.pose.position.x, AMCL_pose_msg->pose.pose.position.y);

            flag_start = true; // start ready
            publishPath();
        } else {
            RCLCPP_WARN(this->get_logger(), "Received null pose message");
        }
    }

    void publishPath(){
        // Check if all messages are received
        //if (true){
        if (flag_dest && flag_start && flag_dest){

            std::cout << "A* planning path with start: x: " << start.x << "  y:" << start.y << std::endl;
            std::cout << "dest is x: " << dest.x << "  y:" << dest.y << std::endl;

            // get path from Astar
            std::list<point *> path_points = myAstar.getPath(start, dest);
            std::cout << "Planning finished" << std::endl;
            
            // show path and insert path to path_msg
            std::string frameID = "map";
            for (auto &p : path_points) {
                std::cout<< "(" << map2poseX(p->x) << "," << map2poseY(p->y) << ")"; // print

                geometry_msgs::msg::PoseStamped pose;
                pose.pose.position.x = map2poseX(p->x);
                pose.pose.position.y = map2poseY(p->y);
                pose.pose.orientation = goal_quat;


                path_msg.poses.push_back(pose);
                path_msg.header.frame_id = "map";
            }
            std::cout << std::endl;
            
            // show path
            //showPath(map, path_points);

            // publish path
            path_publisher_->publish(path_msg);
            std::cout << "myPlanner: path published!" << std::endl;
            // Reset stored messages
            flag_dest = false;
            flag_start = true;
        }
    }


    // Function to convert a 2D vector to an OpenCV Mat object
    void showPath(const std::vector<std::vector<int>>& vec, std::list<point *> path_points) {
        int rows = vec.size();
        int cols = vec[0].size();
        Mat image(rows, cols, CV_8UC1); // Create a grayscale image

        // Loop through the vector and set pixel values
        for (int i = 0; i < rows; ++i) {
            for (int j = 0; j < cols; ++j) {

                // Assign the value from the vector to the corresponding pixel
                image.at<uchar>(i, j) = static_cast<uchar>(vec[i][j]);
            }
        }

        for (auto p : path_points){
            image.at<uchar>(p->x, p->y) = 1;
        }

        // Create a mirrored image
        Mat mirroredImage;
        flip(image, mirroredImage, 1); // Flip horizontally (1)

        // Rotate the image clockwise by 90 degrees
        Mat rotatedImage;
        rotate(mirroredImage, rotatedImage, ROTATE_90_CLOCKWISE);

        imshow("path", rotatedImage);
        waitKey(0);


    }




    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr AMCL_pose_subscription_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;



    // stores message from the subscribers
    std::vector<std::vector<int>> map;
    point start;
    point dest;
    bool flag_map, flag_start, flag_dest = false; // data not ready
    Astar myAstar;
    nav_msgs::msg::Path path_msg;
    geometry_msgs::msg::Quaternion goal_quat;


};

int main(int argc, char** argv) {
    std::cout << "myPlanner is running" << std::endl;
    
    std::cout << std::endl;
    rclcpp::init(argc, argv);
    auto node = std::make_shared<myPlanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}