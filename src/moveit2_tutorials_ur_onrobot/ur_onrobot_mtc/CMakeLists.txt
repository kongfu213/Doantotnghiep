cmake_minimum_required(VERSION 3.8)
project(ur_onrobot_mtc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(shape_msgs REQUIRED)

add_executable(minimal src/minimal.cpp)
target_include_directories(minimal PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(minimal PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(minimal
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
  moveit_task_constructor_core
  rclcpp
  shape_msgs)

add_executable(pick_and_place src/pick_and_place.cpp)
target_include_directories(pick_and_place PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(pick_and_place PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(pick_and_place
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
  moveit_task_constructor_core
  rclcpp
  shape_msgs)
add_executable(terminal_control src/terminal_control.cpp)
target_include_directories(terminal_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(terminal_control PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(terminal_control
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
  moveit_task_constructor_core
  rclcpp
  shape_msgs)
install(TARGETS
  minimal
  pick_and_place
  terminal_control
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
