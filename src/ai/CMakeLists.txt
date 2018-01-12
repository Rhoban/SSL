cmake_minimum_required(VERSION 2.8)
project(ssl_ai)
set (CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  ssl_client tclap RhobanUtils
)

catkin_package(
    INCLUDE_DIRS .
    CATKIN_DEPENDS ssl_client RhobanUtils tclap
)

find_package(Protobuf REQUIRED)

include_directories(${catkin_INCLUDE_DIRS})

set (SOURCES
    AI.cpp
    AIVisionClient.cpp
    AICommander.cpp
    AICommanderSimulation.cpp
    AICommanderReal.cpp
    GameState.cpp
)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/../../bin/)

add_executable(ai ai.cpp ${SOURCES})
target_link_libraries(ai ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES})
