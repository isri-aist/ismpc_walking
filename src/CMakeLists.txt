set(controller_SRC
  Walking_controller.cpp
  FootTrajectory.cpp
  ISMPC_Solver.cpp
  GUI.cpp
  
)

set(controller_HDR
  ../include/Walking_controller.h
  ../include/FootTrajectory.h  
  ../include/ISMPC_Solver.h
  ../include/ControllerConfiguration.h
  ../include/MPC_state.h
)

set(CONTROLLER_NAME ismpc_walking)
add_library(${CONTROLLER_NAME} SHARED ${controller_SRC} ${controller_HDR})

target_include_directories(${CONTROLLER_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_include_directories(${CONTROLLER_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(${CONTROLLER_NAME} PUBLIC mc_rtc::mc_control mc_rtc::mc_rtc_ros)

install(TARGETS ${CONTROLLER_NAME} DESTINATION ${MC_RTC_LIBDIR})

add_controller(Controller${CONTROLLER_NAME} lib.cpp "")
set_target_properties(Controller${CONTROLLER_NAME} PROPERTIES COMPILE_FLAGS "-DWalking_controller_EXPORTS")
target_link_libraries(Controller${CONTROLLER_NAME} PUBLIC ${CONTROLLER_NAME})


