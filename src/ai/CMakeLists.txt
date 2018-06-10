cmake_minimum_required(VERSION 2.8)
project(ssl_ai)
set (CMAKE_CXX_STANDARD 11)
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ")

find_package(catkin REQUIRED COMPONENTS
  ssl_client tclap rhoban_geometry rhoban_graphs
)

catkin_package(
    INCLUDE_DIRS .
    LIBRARIES ssl_ai
    CATKIN_DEPENDS ssl_client rhoban_geometry rhoban_graphs tclap
)

find_package(Protobuf REQUIRED)

include_directories(${catkin_INCLUDE_DIRS} .)

set (SOURCES
    Data.cpp
    game_informations.cpp
    vision/AIVisionClient.cpp
    vision/VisionData.cpp
    com/AICommander.cpp
    com/AICommanderSimulation.cpp
    com/AICommanderReal.cpp
    robot_behavior/factory.cpp
    robot_behavior/example.cpp
    robot_behavior/goalie.cpp
    robot_behavior/patrol.cpp
    robot_behavior/pass.cpp
    robot_behavior/protect_ball.cpp
    # robot_behavior/search_shoot_area.cpp
    robot_behavior/defensor.cpp
    robot_behavior/passive_defensor.cpp
    robot_behavior/robot_follower.cpp
    robot_behavior/concept_proof_spinner.cpp
    robot_behavior/striker.cpp
    robot_behavior/position_follower.cpp
    robot_behavior/do_nothing.cpp
    robot_behavior/a_star_path.cpp
    robot_behavior/consign_follower.cpp
    robot_behavior/robot_behavior.cpp
    robot_behavior/navigation_with_obstacle_avoidance.cpp
    Ai.cpp
    AiData.cpp
    debug.cpp
    manager/Manager.cpp
    manager/thomas.cpp
    manager/adrien/adrien.cpp
    manager/adrien/Manual_adrien.cpp 
    manager/sebastien/sebastien.cpp
    manager/jeremy/jeremy.cpp
    manager/Match.cpp
    manager/Manual.cpp
    manager/factory.cpp
    manager/example.cpp
    manager/example_for_testing_robot_behaviors.cpp
    strategy/Strategy.cpp
    strategy/halt.cpp
    strategy/mur.cpp
    strategy/striker_with_support.cpp
    strategy/union.cpp
    strategy/tare_and_synchronize.cpp
    strategy/from_robot_behavior.cpp
    strategy/placer.cpp
    strategy/prepare_kickoff.cpp
    strategy/example.cpp
    control/robot_control.cpp
    control/robot_control_with_curve.cpp
    control/robot_control_with_position_following.cpp
    control/pid.cpp
    physic/movement_predicted_by_integration.cpp
    physic/movement_with_no_prediction.cpp
    physic/movement_on_new_frame.cpp
    physic/movement_with_temporal_shift.cpp
    physic/MovementSample.cpp
    physic/Movement.cpp
    physic/factory.cpp
    physic/collision.cpp
    math/ContinuousAngle.cpp
    math/curve.cpp
    math/tangents.cpp
    math/vector2d.cpp
    math/position.cpp
    math/matrix2d.cpp
    math/frame_changement.cpp
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
    core/test_collection.cpp
    core/test_print_collection.cpp
    math/test_curve.cpp
    math/test_circular_vector.cpp
    math/test_frame_changement.cpp
    math/test_matching.cpp
    physic/test_MovementSample.cpp
    physic/test_movement_with_no_prediction.cpp
    physic/test_movement_predicted_by_integration.cpp
    physic/test_collision.cpp
    math/test_ContinuousAngle.cpp
    math/test_tangents.cpp
    math/test_vector2d.cpp
    math/test_matrix2d.cpp
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
