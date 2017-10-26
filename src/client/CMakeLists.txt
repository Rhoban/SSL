cmake_minimum_required(VERSION 2.8)
project(ssl_client)

find_package(catkin REQUIRED COMPONENTS
  RhobanUtils
  )

catkin_package(
    INCLUDE_DIRS .
    LIBRARIES ssl_referee
    CATKIN_DEPENDS RhobanUtils
)

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
