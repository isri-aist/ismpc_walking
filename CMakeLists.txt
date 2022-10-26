cmake_minimum_required(VERSION 3.1)

set(CXX_DISABLE_WERROR 1)
set(CMAKE_CXX_STANDARD 17)

set(PROJECT_NAME ismpc_walking)
set(PROJECT_DESCRIPTION "ismpc_walking")
set(PROJECT_URL "")
set(PROJECT_USE_CMAKE_EXPORT TRUE)
set(PROJECT_VERSION 0.1.0)
set(CMAKE_CXX_STANDARD 17)
set(CXX_DISABLE_WERROR 1) # FIXME fix errors and re-enable

include(cmake/base.cmake)

project(${PROJECT_NAME} LANGUAGES CXX VERSION ${PROJECT_VERSION})

# Check if the project is built inside mc_rtc
if(NOT TARGET mc_rtc::mc_control)
  find_package(mc_rtc REQUIRED)
endif()


add_subdirectory(src)

set(AROBASE "@")
configure_file(etc/ismpc_walking.in.yaml "${CMAKE_CURRENT_BINARY_DIR}/etc/ismpc_walking.yaml")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/ismpc_walking.yaml" DESTINATION "${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/etc")

