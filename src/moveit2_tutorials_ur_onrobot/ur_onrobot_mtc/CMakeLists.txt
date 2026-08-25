cmake_minimum_required(VERSION 3.8)
project(ur_onrobot_mtc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ----------------------------------------------------
# 0. Find Packages (Đã bổ sung đầy đủ)
# ----------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(shape_msgs REQUIRED)
# Các thư viện bổ sung cho dual_arm và left/right node
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(moveit_core REQUIRED)

# ----------------------------------------------------
# 1. Executable: minimal
# ----------------------------------------------------
add_executable(minimal src/minimal.cpp)
target_include_directories(minimal PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(minimal PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(minimal
  geometry_msgs moveit_msgs moveit_ros_planning_interface moveit_task_constructor_core rclcpp shape_msgs)

# ----------------------------------------------------
# 2. Executable: pick_and_place
# ----------------------------------------------------
add_executable(pick_and_place src/pick_and_place.cpp)
target_include_directories(pick_and_place PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(pick_and_place PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(pick_and_place
  geometry_msgs moveit_msgs moveit_ros_planning_interface moveit_task_constructor_core rclcpp shape_msgs)

# ----------------------------------------------------
# 3. Executable: terminal_control
# ----------------------------------------------------
add_executable(terminal_control src/terminal_control.cpp)
target_include_directories(terminal_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(terminal_control PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(terminal_control
  geometry_msgs moveit_msgs moveit_ros_planning_interface moveit_task_constructor_core rclcpp shape_msgs)

# ----------------------------------------------------
# 4. Executable: dual_terminal_control
# ----------------------------------------------------
add_executable(dual_terminal_control src/dual_terminal_control.cpp)
target_include_directories(dual_terminal_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(dual_terminal_control PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(dual_terminal_control
  geometry_msgs moveit_msgs moveit_ros_planning_interface rclcpp shape_msgs)

# ----------------------------------------------------
# 5. Executable: left_arm_mtc_node (Tách riêng tay trái)
# ----------------------------------------------------
add_executable(left_arm_mtc_node src/left_arm_mtc_node.cpp)
target_include_directories(left_arm_mtc_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(left_arm_mtc_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(left_arm_mtc_node
  rclcpp moveit_ros_planning_interface moveit_task_constructor_core tf2_geometry_msgs tf2_eigen geometry_msgs moveit_msgs shape_msgs)

# ----------------------------------------------------
# 6. Executable: right_arm_mtc_node (Tách riêng tay phải)
# ----------------------------------------------------
add_executable(right_arm_mtc_node src/right_arm_mtc_node.cpp)
target_include_directories(right_arm_mtc_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(right_arm_mtc_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(right_arm_mtc_node
  rclcpp moveit_ros_planning_interface moveit_task_constructor_core tf2_geometry_msgs tf2_eigen geometry_msgs moveit_msgs shape_msgs)

# ----------------------------------------------------
# 7. Executable: dual_arm_controller
# ----------------------------------------------------
add_executable(dual_arm_controller
  src/dual_arm_controller.cpp
  src/tasks/task_dual_pick.cpp)
target_include_directories(dual_arm_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(dual_arm_controller PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(dual_arm_controller
  rclcpp moveit_ros_planning_interface moveit_task_constructor_core geometry_msgs)

# ----------------------------------------------------
# 8. Executable: dual_arm_mtc_node
# ----------------------------------------------------
add_executable(dual_arm_mtc_node src/dual_arm_mtc_node.cpp)
target_include_directories(dual_arm_mtc_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(dual_arm_mtc_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(dual_arm_mtc_node
  rclcpp moveit_core moveit_ros_planning_interface moveit_task_constructor_core tf2_geometry_msgs tf2_eigen)

# ----------------------------------------------------
# Installation Rules (Gom gọn toàn bộ vào 1 chỗ duy nhất)
# ----------------------------------------------------
install(TARGETS
  minimal
  pick_and_place
  terminal_control
  dual_terminal_control
  left_arm_mtc_node
  right_arm_mtc_node
  dual_arm_controller
  dual_arm_mtc_node
  DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS
  scripts/spawn_environment.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME})

# ----------------------------------------------------
# Testing
# ----------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()