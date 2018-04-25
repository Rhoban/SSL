cmake_minimum_required(VERSION 2.8)
project(ssl_ai)
set (CMAKE_CXX_STANDARD 11)
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ")

find_package(catkin REQUIRED COMPONENTS
  ssl_client tclap rhoban_geometry eigen
)

catkin_package(
    INCLUDE_DIRS .
    LIBRARIES ssl_ai
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
    robot_behavior/goalie.cpp
    robot_behavior/position_follower.cpp
    robot_behavior/shooter.cpp
    robot_behavior/do_nothing.cpp
    robot_behavior/robot_behavior.cpp
    Ai.cpp
    AiData.cpp
    manager/Manager.cpp
    strategy/Strategy.cpp
    strategy/halt.cpp
    strategy/sandbox.cpp
    strategy/tare_and_synchronize.cpp
    control/robot_control.cpp
    control/robot_control_with_curve.cpp
    control/robot_control_with_position_following.cpp
    control/pid.cpp
    physic/movement_predicted_by_integration.cpp
    physic/movement_with_no_prediction.cpp
    physic/MovementSample.cpp
    physic/Movement.cpp
    math/eigen_convertion.cpp
    math/ContinuousAngle.cpp
    math/curve.cpp
    math/tangents.cpp
    math/vector.cpp
    core/export_to_plot.cpp
    core/logger.cpp
    referee/Referee.cpp
    annotations/Annotations.cpp
)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/../../bin/)
set(TEST_DIRECTORY ${CMAKE_SOURCE_DIR}/../../bin/)

set(ALL_LIBS ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES})

add_library (ssl_ai ${SOURCES})
target_link_libraries(ssl_ai ${ALL_LIBS})

add_executable(ai main.cpp)
target_link_libraries(ai ssl_ai ${ALL_LIBS})

message(WARNING "CATKIN ENABLE TESTING: ${CATKIN_ENABLE_TESTING}")

if(CATKIN_ENABLE_TESTING)
  include(CTest)

  set (TEST_SOURCES
    core/test_machine_state.cpp
    core/test_print_collection.cpp
    math/test_eigen_convertion.cpp
    math/test_curve.cpp
    math/test_circular_vector.cpp
    physic/test_MovementSample.cpp
    physic/test_movement_with_no_prediction.cpp
    physic/test_movement_predicted_by_integration.cpp
    math/test_ContinuousAngle.cpp
    math/test_tangents.cpp
    math/test_vector.cpp
    control/test_pid.cpp
    control/test_robot_control.cpp
    robot_behavior/test_robot_behavior.cpp
  )

  foreach(test_source ${TEST_SOURCES})
      get_filename_component(name_test ${test_source} NAME_WE)
      catkin_add_gtest(${name_test} ${test_source})
      target_link_libraries(${name_test} ssl_ai ${ALL_LIBS})
  endforeach(test_source)
endif(CATKIN_ENABLE_TESTING)
