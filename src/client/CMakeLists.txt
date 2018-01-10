cmake_minimum_required(VERSION 2.8)
project(ssl_client)

find_package(catkin REQUIRED COMPONENTS
  RhobanUtils
  serial
  )

catkin_package(
    INCLUDE_DIRS . ${CMAKE_CURRENT_BINARY_DIR}
    LIBRARIES ssl_client
    CATKIN_DEPENDS RhobanUtils serial
)

# This is not really clean but will produce the binaries in the workspace
# directory
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/../../bin/)

find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_BINARY_DIR})

PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS
    proto/referee.proto
    proto/messages_robocup_ssl_geometry.proto
    proto/messages_robocup_ssl_detection.proto
    proto/messages_robocup_ssl_wrapper.proto
    proto/grSim_Commands.proto
    proto/grSim_Replacement.proto
    proto/grSim_Packet.proto
)

set (SOURCES
    MulticastClient.cpp
    RefereeClient.cpp
    VisionClient.cpp
    SimClient.cpp
    Kinematic.cpp
    Master.cpp
    ${PROTO_SRCS}
    ${PROTO_HDRS}
)

add_library(ssl_client ${SOURCES})
target_link_libraries (ssl_client ${PROTOBUF_LIBRARIES} ${catkin_LIBRARIES})

add_executable (referee referee.cpp ${SOURCES})
target_link_libraries (referee ssl_client)

add_executable (vision vision.cpp ${SOURCES})
target_link_libraries (vision ssl_client)

add_executable (sim sim.cpp ${SOURCES})
target_link_libraries (sim ssl_client)

add_executable (kinematic kinematic.cpp ${SOURCES})
target_link_libraries (kinematic ssl_client)

add_executable (master master.cpp ${SOURCES})
target_link_libraries (master ssl_client)
