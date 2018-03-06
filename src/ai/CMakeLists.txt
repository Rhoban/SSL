cmake_minimum_required(VERSION 2.8)
project(ssl_ai)
set (CMAKE_CXX_STANDARD 11)

set(CMAKE_BUILD_TYPE Debug)

include(CTest)
enable_testing()

find_package(catkin REQUIRED COMPONENTS
  ssl_client tclap rhoban_geometry eigen
)

catkin_package(
    INCLUDE_DIRS .
    CATKIN_DEPENDS ssl_client rhoban_geometry tclap eigen
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
    ai/robot_behavior/goalie.cpp
    ai/robot_behavior/position_follower.cpp
    ai/robot_behavior/shooter.cpp
    ai/robot_behavior/robot_behavior.cpp
    ai/robot_behavior/do_nothing.cpp
    ai/AI.cpp
    ai/AiData.cpp
    control/robot_control.cpp
    control/robot_control_with_curve.cpp
    control/robot_control_with_position_following.cpp
    control/pid.cpp
    physic/movement_predicted_by_integration.cpp
    physic/movement_with_no_prediction.cpp
    physic/MovementSample.cpp
    physic/Movement.cpp
    math/ContinuousAngle.cpp
    math/curve.cpp
    referee/Referee.cpp
)

set (TEST_SOURCES
    core/test_machine_state.cpp
    core/test_print_collection.cpp
    math/test_curve.cpp
    math/test_circular_vector.cpp
    physic/test_MovementSample.cpp
    physic/test_movement_with_no_prediction.cpp
    physic/test_movement_predicted_by_integration.cpp
    math/test_ContinuousAngle.cpp
    control/test_pid.cpp
    control/test_robot_control.cpp
)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/../../bin/)
set(TEST_DIRECTORY ${CMAKE_SOURCE_DIR}/../../bin/)

add_executable(ai main.cpp ${SOURCES})
target_link_libraries(ai ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES})


#foreach(test_source ${TEST_SOURCES})
#    get_filename_component(name_test ${test_source} NAME_WE)
##    get_filename_component(name_extention_test ${test_source} NAME)
##    get_filename_component(directory_test ${test_source} DIRECTORY)
##    string(REPLACE "test_" ${directory_test}/ source_test ${name_extention_test}) 
##    string(REPLACE ".cpp" ".h" header1_test ${source_test}) 
##    string(REPLACE ".cpp" ".hpp" header2_test ${source_test}) 
##
##    if(EXISTS ${CMAKE_SOURCE_DIR}/${source_test})
##        add_executable(${name_test} ${test_source} ${source_test})
##    elseif(EXISTS ${CMAKE_SOURCE_DIR}/${header1_test})
##        add_executable(${name_test} ${test_source})
##    elseif(EXISTS ${CMAKE_SOURCE_DIR}/${header2_test})
##        add_executable(${name_test} ${test_source})
##    else(EXISTS ${CMAKE_SOURCE_DIR}/${source_test})
#        add_executable(${name_test} ${test_source} ${SOURCES})
##    endif(EXISTS ${CMAKE_SOURCE_DIR}/${source_test})
#
#    add_test(NAME ${name_test} COMMAND ./${name_test} WORKING_DIRECTORY ${TEST_DIRECTORY})
#    target_link_libraries(${name_test} stdc++fs ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES})
#endforeach(test_source)
