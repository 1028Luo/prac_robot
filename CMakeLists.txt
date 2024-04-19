cmake_minimum_required(VERSION 3.8)
project(prac_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED) # for myPlanner
find_package(nav_msgs REQUIRED) # for myPlanner
find_package(geometry_msgs REQUIRED) # for myPlanner
find_package(Eigen3 REQUIRED) 
include_directories(${EIGEN3_INCLUDE_DIR})
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)

  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


# Add executable #line 26, 27, 31 # for myPlanner need to match
add_executable(myPlanner 
  src/myPlanner.cpp
  src/simple_Astar.cpp)
ament_target_dependencies(myPlanner rclcpp nav_msgs geometry_msgs tf2)


add_executable(myController
  src/myController.cpp
  src/simple_LQR.cpp)
ament_target_dependencies(myController rclcpp nav_msgs geometry_msgs tf2)

add_executable(myMPC
  src/myMPC.cpp
  src/simple_LQR.cpp)
ament_target_dependencies(myMPC rclcpp nav_msgs geometry_msgs tf2)


install(TARGETS
  myPlanner
  myController
  myMPC
  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY
  launch description config maps
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
