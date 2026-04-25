cmake_minimum_required(VERSION 3.8)
project(effective_mass_stats_vis)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(assimp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)

add_executable(generic_point_effective_mass_visualization src/generic_point_effective_mass_visualization.cpp)
target_include_directories(generic_point_effective_mass_visualization PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(generic_point_effective_mass_visualization assimp::assimp pinocchio::pinocchio)

ament_target_dependencies(generic_point_effective_mass_visualization
  rclcpp
  sensor_msgs
  visualization_msgs
  ament_index_cpp
  geometry_msgs
  std_msgs
  Eigen3
  pinocchio
)

install(TARGETS generic_point_effective_mass_visualization
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
