cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fmessage-length=0")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fmessage-length=0")

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_subdirectory(model_ctl)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/model_ctl)

# compile plugin
add_library(repl_control SHARED plugin.cpp)
target_link_libraries(repl_control model_ctl ${GAZEBO_LIBRARIES})

# compile command client
add_executable(repl_client client.cpp)
target_link_libraries(repl_client model_ctl ${GAZEBO_LIBRARIES})

# compile script client
add_executable(repl_script script.cpp)
target_link_libraries(repl_script model_ctl ${GAZEBO_LIBRARIES})