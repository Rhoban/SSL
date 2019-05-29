cmake_minimum_required(VERSION 2.8)
project(ssl_client)

find_package(catkin REQUIRED COMPONENTS
  rhoban_geometry
  serial
  jsoncpp
  )

catkin_package(
    INCLUDE_DIRS . ${CMAKE_CURRENT_BINARY_DIR}
    LIBRARIES ssl_client
    CATKIN_DEPENDS rhoban_geometry serial jsoncpp
)

# This is not really clean but will produce the binaries in the workspace
# directory
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/../../bin/)
set (CMAKE_CXX_STANDARD 11)

find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_BINARY_DIR})

PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS
    grproto/messages_robocup_ssl_geometry.proto
    grproto/messages_robocup_ssl_detection.proto
    grproto/messages_robocup_ssl_wrapper.proto
    grproto/grSim_Commands.proto
    grproto/grSim_Replacement.proto
    grproto/grSim_Packet.proto
    refproto/ssl_game_controller_common.proto
    refproto/ssl_game_event_2019.proto
    refproto/ssl_game_event.proto
    refproto/ssl_referee.proto
)

set (SOURCES
    multicast_client_single_thread.cpp
    execution_manager.cpp
    MulticastClient.cpp
    RefereeClient.cpp
    referee_client_single_thread.cpp
    VisionClient.cpp
    viewer_server.cpp
    SimClient.cpp
    Kinematic.cpp
    Master.cpp
    joystick/Joystick.cpp
    ${PROTO_SRCS}
    ${PROTO_HDRS}
)

add_library(ssl_client ${SOURCES})
target_link_libraries (ssl_client ${PROTOBUF_LIBRARIES} ${catkin_LIBRARIES} websockets)
target_include_directories(ssl_client PUBLIC ${CMAKE_SOURCE_DIR})

add_executable (referee referee.cpp ${SOURCES})
target_link_libraries (referee ssl_client)

add_executable (referee_st referee_st.cpp ${SOURCES})
target_link_libraries (referee_st ssl_client)

add_executable (vision vision.cpp ${SOURCES})
target_link_libraries (vision ssl_client)

add_executable (vision_st vision_st.cpp ${SOURCES})
target_link_libraries (vision_st ssl_client)

add_executable (sim sim.cpp ${SOURCES})
target_link_libraries (sim ssl_client)

add_executable (kinematic kinematic.cpp ${SOURCES})
target_link_libraries (kinematic ssl_client)

add_executable (master master.cpp ${SOURCES})
target_link_libraries (master ssl_client)

add_executable (viewer_threaded viewer_threaded.cpp ${SOURCES})
target_link_libraries (viewer_threaded ssl_client)

message(WARNING "CATKIN ENABLE TESTING: ${CATKIN_ENABLE_TESTING}")

if(CATKIN_ENABLE_TESTING)
  include(CTest)

  
  if(ENABLE_COVERAGE_TESTING)
    find_package(code_coverage REQUIRED)
    include(CodeCoverage)
    APPEND_COVERAGE_COMPILER_FLAGS()
  endif()
  
  set (TEST_SOURCES
    tests/test_execution_manager.cpp
    )
  
  foreach(test_source ${TEST_SOURCES})
    get_filename_component(name_test ${test_source} NAME_WE)
    catkin_add_gtest(${name_test} ${test_source})
    target_link_libraries(${name_test} ssl_client)
  endforeach(test_source)

  if(ENABLE_COVERAGE_TESTING)
    set(COVERAGE_EXCLUDES "*/${PROJECT_NAME}/test*")
    add_code_coverage(
      NAME ${PROJECT_NAME}_coverage
      DEPENDS tests
      )
  endif()
endif(CATKIN_ENABLE_TESTING)
