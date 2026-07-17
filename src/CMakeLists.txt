set(CONTROLLER_NAME ismpc_walking)
add_library(${CONTROLLER_NAME} SHARED)
target_sources(
  ${CONTROLLER_NAME}
  PRIVATE
  Walking_controller.cpp
  move_feet.cpp
  robot_state.cpp
  FootTrajectory.cpp
  ISMPC_Solver.cpp
  GUI.cpp
  logs.cpp
  joystick.cpp
  Polygon.cpp
)

jrl_target_headers(
  ${CONTROLLER_NAME}
  PUBLIC HEADERS
  ../include/ismpc_walking/Walking_controller.h
  ../include/ismpc_walking/FootTrajectory.h  
  ../include/ismpc_walking/ISMPC_Solver.h
  ../include/ismpc_walking/ControllerConfiguration.h
  ../include/ismpc_walking/MPC_state.h
  ../include/ismpc_walking/Polygon.h
  BASE_DIRS
  ../include
)

target_include_directories(${CONTROLLER_NAME} PUBLIC $<INSTALL_INTERFACE:include> $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>)
target_link_libraries(${CONTROLLER_NAME} PUBLIC mc_rtc::mc_control_fsm mc_rtc::mc_rtc_ros)
target_link_libraries(${CONTROLLER_NAME} PUBLIC pendulum_feasibility_solver::pendulum_feasibility_solver mc_joystick_plugin::joystick_inputs)

jrl_add_export_component(NAME ${CONTROLLER_NAME} TARGETS ${CONTROLLER_NAME})

add_controller(${CONTROLLER_NAME}_controller lib.cpp "")
set_target_properties(${CONTROLLER_NAME}_controller PROPERTIES OUTPUT_NAME "${CONTROLLER_NAME}")
target_link_libraries(${CONTROLLER_NAME}_controller PUBLIC ${CONTROLLER_NAME})

install(TARGETS ${CONTROLLER_NAME}
  EXPORT "${TARGETS_EXPORT_NAME}"
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

install(
  DIRECTORY ${CMAKE_SOURCE_DIR}/include/ DESTINATION include)

add_subdirectory(states)
