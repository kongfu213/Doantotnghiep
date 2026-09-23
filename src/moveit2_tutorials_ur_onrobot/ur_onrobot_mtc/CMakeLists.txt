cmake_minimum_required(VERSION 3.8)
project(ur_onrobot_mtc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(${PROJECT_NAME}_core
  src/dual_arm_interface.cpp
  src/tasks/robot_selector.cpp
  src/tasks/pick_task.cpp
  src/tasks/place_task.cpp
  src/tasks/move_task.cpp
  src/tasks/mission_task.cpp
  src/tasks/stack_task.cpp
  src/tasks/swap_task.cpp
)

target_include_directories(${PROJECT_NAME}_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_core
  rclcpp
  std_msgs
  geometry_msgs
  shape_msgs
  moveit_msgs
  moveit_core
  moveit_ros_planning_interface
  moveit_task_constructor_core
  visualization_msgs
)

target_link_libraries(${PROJECT_NAME}_core Eigen3::Eigen)

add_executable(dual_arm_mtc_node
  src/dual_arm_mtc_node.cpp
)

target_link_libraries(dual_arm_mtc_node
  ${PROJECT_NAME}_core
)

ament_target_dependencies(dual_arm_mtc_node
  rclcpp
  std_msgs
)


add_executable(setup_pcb_scene
  src/setup_pcb_scene.cpp
)

ament_target_dependencies(setup_pcb_scene
  rclcpp
  geometry_msgs
  shape_msgs
  moveit_msgs
  moveit_ros_planning_interface
  visualization_msgs
)

add_executable(robot_cli
  src/robot_cli.cpp
)

ament_target_dependencies(robot_cli
  rclcpp
  std_msgs
)

install(
  TARGETS
    ${PROJECT_NAME}_core
    dual_arm_mtc_node
    robot_cli
    setup_pcb_scene
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

# Keep using the existing spawn_environment.py from your package.
# If it is in scripts/, install it as an executable as below.
install(
  PROGRAMS
    scripts/spawn_environment.py
  DESTINATION lib/${PROJECT_NAME}
  OPTIONAL
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  std_msgs
  geometry_msgs
  shape_msgs
  moveit_msgs
  moveit_core
  moveit_ros_planning_interface
  moveit_task_constructor_core
  visualization_msgs
)

ament_package()
