cmake_minimum_required(VERSION 2.8)
project(ssl_ai)
set (CMAKE_CXX_STANDARD 11)

set(CMAKE_BUILD_TYPE Debug)

include(CTest)
enable_testing()

find_package(catkin REQUIRED COMPONENTS
  ssl_client tclap RhobanUtils Eigen
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
    movment.cpp
)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/../../bin/)

add_executable(ai ai.cpp ${SOURCES})
target_link_libraries(ai ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES})

add_executable(test_movment test_movment.cpp movment.cpp)
add_test(NAME test_movment COMMAND ./test_movment)
