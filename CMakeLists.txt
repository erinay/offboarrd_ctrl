cmake_minimum_required(VERSION 3.8)
project(offboard_ctrl)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(px4_ros_com REQUIRED)

# ======== ACADOS PATHS ========
set(ACADOS_DIR /home/erina/acados)
set(MPC_CODE_DIR ${CMAKE_SOURCE_DIR}/../mpc_solver/c_generated_code)

include_directories(
  ${ACADOS_DIR}/include
  ${ACADOS_DIR}/include/acados
  ${ACADOS_DIR}/include/blasfeo/include
  ${ACADOS_DIR}/include/hpipm/include
  ${MPC_CODE_DIR}
)

link_directories(
  ${ACADOS_DIR}/lib
  ${MPC_CODE_DIR}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(offboard_attitude src/offboard_attitude.cpp)
ament_target_dependencies(offboard_attitude rclcpp px4_msgs px4_ros_com)
install(TARGETS offboard_attitude DESTINATION lib/${PROJECT_NAME})

target_link_libraries(offboard_attitude
  "${MPC_CODE_DIR}/libacados_ocp_solver_single_integrator.so"
  "${MPC_CODE_DIR}/libacados_ocp_solver_double_integrator.so"
  acados
)


ament_package()
