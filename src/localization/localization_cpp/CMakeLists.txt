cmake_minimum_required(VERSION 3.8)
project(localization_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(erp42_msgs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(Eigen3 REQUIRED)

# 권장: 타겟 기반 include (또는 아래의 include_directories 대체)
# target_include_directories(rotate_yaw PRIVATE ${EIGEN3_INCLUDE_DIRS})

add_executable(rotate_yaw src/rotate_yaw.cpp)
ament_target_dependencies(rotate_yaw
  rclcpp
  std_msgs
  sensor_msgs
  erp42_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
)
# Eigen을 타겟로 링크 (둘 중 하나 택1)
target_link_libraries(rotate_yaw Eigen3::Eigen)
# 또는 include 경로만 추가하고 싶다면:
# target_include_directories(rotate_yaw PRIVATE ${EIGEN3_INCLUDE_DIRS})

add_executable(gps_dummy src/gps_dummy.cpp)
ament_target_dependencies(gps_dummy
  rclcpp
  std_msgs
  sensor_msgs
  erp42_msgs
  nav_msgs
  geometry_msgs
)

install(TARGETS
  rotate_yaw
  gps_dummy
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

