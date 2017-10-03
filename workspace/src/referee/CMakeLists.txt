cmake_minimum_required(VERSION 2.8)
project(ssl_referee)

find_package(catkin REQUIRED COMPONENTS
  ssl_refbox
  )

find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_BINARY_DIR})

PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${ssl_refbox_INCLUDE_DIRS}/referee.proto)
message(${ssl_refbox_INCLUDE_DIRS})

set (SOURCES
    main.cpp
    ${PROTO_SRCS}
    ${PROTO_HDRS}
)

add_executable (referee ${SOURCES})
target_link_libraries (referee ${PROTOBUF_LIBRARIES})
