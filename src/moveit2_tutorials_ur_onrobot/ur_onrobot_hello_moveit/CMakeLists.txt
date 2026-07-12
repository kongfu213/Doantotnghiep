cmake_minimum_required(VERSION 3.8)
project(ur_onrobot_hello_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(rclcpp REQUIRED)
find_package(shape_msgs REQUIRED)

add_executable(your_first_project src/your_first_project.cpp)
target_compile_features(your_first_project PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(your_first_project
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
  rclcpp
  shape_msgs)

add_executable(visualizing_in_rviz src/visualizing_in_rviz.cpp)
target_compile_features(visualizing_in_rviz PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(visualizing_in_rviz
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
  moveit_visual_tools
  rclcpp
  shape_msgs)

add_executable(planning_around_objects src/planning_around_objects.cpp)
target_compile_features(planning_around_objects PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(planning_around_objects
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
  moveit_visual_tools
  rclcpp
  shape_msgs)

install(TARGETS
  your_first_project
  visualizing_in_rviz
  planning_around_objects
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
