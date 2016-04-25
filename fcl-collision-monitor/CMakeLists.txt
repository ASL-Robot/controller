cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fmessage-length=0")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fmessage-length=0")
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}")

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_subdirectory(../repl-control/model_ctl ${CMAKE_CURRENT_BINARY_DIR}/model_ctl)
add_subdirectory(../fcl-collisions ${CMAKE_CURRENT_BINARY_DIR}/fcl-collisions)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/model_ctl ../repl-control ../fcl-collisions)

# compile client program
add_executable(collision_monitor collide-mon.cpp)
target_link_libraries(collision_monitor model_ctl fcl_robot_s ${GAZEBO_LIBRARIES})

