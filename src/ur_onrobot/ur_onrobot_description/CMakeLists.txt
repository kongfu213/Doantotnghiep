cmake_minimum_required(VERSION 3.8)
project(ur_onrobot_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Tìm các gói phụ thuộc bắt buộc của ROS 2
find_package(ament_cmake REQUIRED)
find_package(ur_description REQUIRED)
find_package(onrobot_description REQUIRED)

# ĐĂNG KÝ CÀI ĐẶT CÁC THƯ MỤC CHỨA FILE URDF, LAUNCH, RVIZ SANG THƯ MỤC INSTALL
install(DIRECTORY launch rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package() 