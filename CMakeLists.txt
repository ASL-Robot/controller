cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fmessage-length=0")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fmessage-length=0")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/out")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/out")

file(COPY model/model.config DESTINATION out)
file(COPY model/robot.sdf    DESTINATION out)
file(COPY model/robot.world  DESTINATION out)
file(COPY model/words        DESTINATION out)
#add_custom_target(COPY_MODEL_TO_OUTPUT ALL
#	COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/model/model.config ${CMAKE_CURRENT_BINARY_DIR}/out/
#	COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/model/robot.sdf    ${CMAKE_CURRENT_BINARY_DIR}/out/
#	COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/model/robot.world  ${CMAKE_CURRENT_BINARY_DIR}/out/
#	COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/model/words ${CMAKE_CURRENT_BINARY_DIR}/out/)

set(COMBINED_BUILD ON)
message(STATUS "Configuring FCL collision checker ...")
add_subdirectory(fcl-collisions)
message(STATUS "Configuring Gazebo REPL client ...")
add_subdirectory(repl-control)
message(STATUS "Configuring FCL collider to Gazebo bridge ...")
add_subdirectory(fcl-collision-monitor)
