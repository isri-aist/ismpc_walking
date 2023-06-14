cmake_minimum_required(VERSION 3.1)

set(CXX_DISABLE_WERROR 1)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)

set(PROJECT_NAME ismpc_walking)
set(PROJECT_VERSION 0.1.0)
set(PROJECT_DESCRIPTION "ismpc_walking")
set(PROJECT_URL "")

# jrl-cmakemodules minimal settings
set(PROJECT_USE_CMAKE_EXPORT TRUE)
set(INSTALL_DOCUMENTATION OFF CACHE BOOL "" FORCE)
set(INSTALL_GENERATED_HEADERS OFF CACHE BOOL "" FORCE)
set(INSTALL_GENERATED_HEADERS OFF CACHE BOOL "" FORCE)
set(CXX_DISABLE_WERROR 1) # FIXME fix errors and re-enable

include(cmake/base.cmake)

project(${PROJECT_NAME} LANGUAGES CXX VERSION ${PROJECT_VERSION})

# Check if the project is built inside mc_rtc
if(NOT TARGET mc_rtc::mc_control)
  add_project_dependency(mc_rtc REQUIRED)
endif()

add_project_dependency(pendulum_feasibility_solver REQUIRED)
# find_package(FootStepsPlannerPlugin REQUIRED)

add_subdirectory(src)

set(AROBASE "@")
configure_file(etc/ismpc_walking.in.yaml "${CMAKE_CURRENT_BINARY_DIR}/etc/ismpc_walking.yaml")
install_controller_configuration("${CMAKE_CURRENT_BINARY_DIR}/etc/ismpc_walking.yaml")
foreach(ROBOT hrp4cr hrp4 hrp2_drc hrp5_p rhps1)
  install_controller_robot_configuration(ismpc_walking etc/robots/${ROBOT}.yaml)
endforeach()
