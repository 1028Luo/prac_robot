cmake_minimum_required(VERSION 3.8)
project(prac_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED) # for mapProcesser
find_package(nav_msgs REQUIRED) # for mapProcesser



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)

  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


# Add executable
add_executable(map_processor src/mapProcessor.cpp)
ament_target_dependencies(map_processor rclcpp nav_msgs)


install(TARGETS
  mapProcessor
  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY
  launch description config maps
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
