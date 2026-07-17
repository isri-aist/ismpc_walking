cmake_minimum_required(VERSION 3.22)

include(cmake/get-jrl-cmakemodules.cmake)

project(ismpc_walking 
VERSION 0.1.0
DESCRIPTION "ismpc_walking"
HOMEPAGE_URL "https://github.com/isri-aist/ismpc_walking.git"
LANGUAGES CXX)

jrl_configure_defaults()

# Check if the project is built inside mc_rtc
if(NOT TARGET mc_rtc::mc_control)
  jrl_find_package(mc_rtc REQUIRED EXPECTED_TARGETS mc_rtc::mc_control mc_rtc::mc_rtc_ros)
endif()

jrl_find_package(pendulum_feasibility_solver REQUIRED EXPECTED_TARGETS pendulum_feasibility_solver::pendulum_feasibility_solver)
jrl_find_package(mc_joystick_plugin REQUIRED EXPECTED_TARGETS mc_joystick_plugin::joystick_inputs)

add_subdirectory(src)

set(AROBASE "@")
configure_file(etc/ismpc_walking.in.yaml "${CMAKE_CURRENT_BINARY_DIR}/etc/ismpc_walking.yaml")
install_controller_configuration("${CMAKE_CURRENT_BINARY_DIR}/etc/ismpc_walking.yaml")
foreach(ROBOT hrp4cr hrp4 hrp2_drc hrp5_p rhps1 jvrc1)
  install_controller_robot_configuration(ismpc_walking etc/robots/${ROBOT}.yaml)
endforeach()
