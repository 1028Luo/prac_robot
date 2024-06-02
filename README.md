This project is a simulation of a simple differential drive robot using Gazebo and ROS2. I built it from scratch and was using it to learn and practice as a robotics engineer.  

Some things you can do with the robot:

make some gif here


A breakdown of the project and some know-hows (I use this as a recap):


1. simulation building using Gazebo fortress and ROS2 humble
    1.1 Defining robot model using URDF and Xacro
    1.2 Application of diffrential drive control package from ROS2 control 
    1.3 Simple sensor fusion

2. Exploration mode

3. Navigation mode

    3.1 Localsization
        3.1.1 Setting up frames and tf

        3.1.2 

    3.2 Custom global path planner 
        3.2.1 implementation of A* and RRT on odccupancy grid map
        3.2.2 

    3.3 Custom controller

4. Wrapping everything in Docker

More content coming soon