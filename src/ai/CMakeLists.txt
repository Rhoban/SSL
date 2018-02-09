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

include_directories(${catkin_INCLUDE_DIRS} .)

set (SOURCES
    Data.cpp
    vision/AIVisionClient.cpp
    vision/VisionData.cpp
    com/AICommander.cpp
    com/AICommanderSimulation.cpp
    com/AICommanderReal.cpp
    ai/robot_behavior.cpp
    ai/AI.cpp
    ai/AiData.cpp
    control/robot_control.cpp
    control/pid.cpp
    control/curve.cpp
    tools/Movement.cpp
    tools/MovementSample.cpp
    tools/MovementSample.cpp
    prediction/movement_prediction.cpp
    referee/Referee.cpp
)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/../../bin/)

add_executable(ai main.cpp ${SOURCES})
target_link_libraries(ai stdc++fs ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES})

add_executable(test_curve control/test_curve.cpp control/curve.cpp)
add_test(NAME test_curve COMMAND ./test_curve)
add_executable(test_circular_vector tools/test_circular_vector.cpp)
add_test(NAME test_circular_vector COMMAND ./test_circular_vector)
