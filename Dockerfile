FROM ros:humble

# Update and install development tools
RUN apt-get update && apt-get install -y \
    build-essential \
    python3-colcon-common-extensions \
    python3-rosdep \
    libopencv-dev \
    ros-humble-xacro \
    ros-humble-ros-gz\
    ros-humble-teleop-twist-keyboard \
    xterm \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \  
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-ign-ros2-control\
    && rm -rf /var/lib/apt/lists/*

RUN rosdep update

WORKDIR /ros2_ws/src
#COPY prac_robot ./prac_robot

WORKDIR /ros2_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.sh && \
                    apt-get update && \
                    rosdep install --from-paths src --ignore-src -r -y && \
                    colcon build"
RUN . /opt/ros/humble/setup.sh
RUN colcon build 
