cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fmessage-length=0 -fPIC")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fmessage-length=0 -fPIC")
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}")
message(STATUS "CMAKE_MODULE_PATH = ${CMAKE_MODULE_PATH}")

#find_package(FCL 0.4.0 REQUIRED) # Why isn't this working?

include_directories("${CMAKE_CURRENT_SOURCE_DIR}")
add_library(fcl_robot SHARED RobotModel.cpp CollisionModel.cpp KinematicModel.cpp)
target_link_libraries(fcl_robot fcl)
add_library(fcl_robot_s STATIC RobotModel.cpp CollisionModel.cpp KinematicModel.cpp)
target_link_libraries(fcl_robot_s fcl)

