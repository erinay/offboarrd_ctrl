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
find_package(nav_msgs REQUIRED)

option(BUILD_ACADOS_CONTROLLER
  "Build the ACADOS-based offboard_attitude controller" OFF)

if(BUILD_ACADOS_CONTROLLER)
  set(ACADOS_DIR "/home/erina/acados" CACHE PATH
    "Path to the ACADOS installation")
  set(MPC_CODE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../mpc_solver/c_generated_code" CACHE PATH
    "Path to the generated MPC solver code")
endif()


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

add_executable(autonomy_controller src/autonomy_controller.cpp)
add_executable(offboard_follower src/offboard_follower.cpp)
ament_target_dependencies(autonomy_controller rclcpp px4_msgs nav_msgs)
ament_target_dependencies(offboard_follower rclcpp px4_msgs geometry_msgs nav_msgs)
target_link_libraries(offboard_follower Eigen3::Eigen)

install(TARGETS autonomy_controller offboard_follower DESTINATION lib/${PROJECT_NAME})

if(BUILD_ACADOS_CONTROLLER)
  add_executable(offboard_attitude src/offboard_attitude.cpp)
  target_include_directories(offboard_attitude PRIVATE
    ${ACADOS_DIR}/include
    ${ACADOS_DIR}/include/acados
    ${ACADOS_DIR}/include/blasfeo/include
    ${ACADOS_DIR}/include/hpipm/include
    ${MPC_CODE_DIR}
  )
  target_link_directories(offboard_attitude PRIVATE
    ${ACADOS_DIR}/lib
    ${MPC_CODE_DIR}
  )
  ament_target_dependencies(offboard_attitude rclcpp px4_msgs)
  target_link_libraries(offboard_attitude
    "${MPC_CODE_DIR}/libacados_ocp_solver_single_integrator.so"
    "${MPC_CODE_DIR}/libacados_ocp_solver_double_integrator.so"
    acados
  )
  install(TARGETS offboard_attitude DESTINATION lib/${PROJECT_NAME})
endif()


ament_package()
