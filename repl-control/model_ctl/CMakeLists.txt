find_package(Protobuf REQUIRED)
find_package(Gazebo REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

set(PROTOBUF_IMPORT_DIRS)
foreach (ITR ${GAZEBO_INCLUDE_DIRS})
    if (ITR MATCHES ".*gazebo-[0-9.]+$")
        set(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
    endif()
endforeach()

set(msgs model_ctl.proto ${PROTOBUF_IMPORT_DIRS}/time.proto)
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${msgs})
set_source_files_properties(${PROTO_SRCS} ${PROTO_HDRS} PROPERTIES GENERATED ON)

include_directories(${CMAKE_CURRENT_BINARY_DIR})
add_library(model_ctl STATIC ${PROTO_SRCS} ModelController.cpp ModelScripter.cpp)
target_link_libraries(model_ctl ${PROTOBUF_LIBRARY} ${GAZEBO_LIBRARIES})