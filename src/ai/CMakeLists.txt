cmake_minimum_required(VERSION 2.8)
project(ssl_ai)
set (CMAKE_CXX_STANDARD 11)
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ")

find_package(catkin REQUIRED COMPONENTS
  ssl_client tclap rhoban_geometry jsoncpp
)

catkin_package(
    INCLUDE_DIRS .
    LIBRARIES ssl_ai
    CATKIN_DEPENDS ssl_client rhoban_geometry tclap jsoncpp
)

find_package(Protobuf REQUIRED)

include_directories(${PROTOBUF_INCLUDE_DIRS} .)
include_directories(${catkin_INCLUDE_DIRS} .)
include_directories(${CMAKE_CURRENT_BINARY_DIR})

set (SOURCES
    data.cpp
    data/ball.cpp
    data/robot.cpp
    data/field.cpp
    data/camera_data.cpp
    data/mobile.cpp
    data/referee.cpp
    data/ai_data.cpp
    data/computed_data.cpp
    config.cpp
    game_informations.cpp
    vision/ai_vision_client.cpp
    vision/vision_data.cpp
    vision/print_protobuf.cpp
    vision/robot_position_filter.cpp
    vision/factory.cpp
    vision/time_synchroniser.cpp
    com/ai_commander.cpp
    viewer/viewer_communication.cpp
    viewer/properties.cpp
    robot_behavior/factory.cpp
    robot_behavior/do_nothing.cpp
    robot_behavior/position_follower.cpp
    robot_behavior/consign_follower.cpp
    robot_behavior/robot_behavior.cpp
    robot_behavior/navigation_with_obstacle_avoidance.cpp
    robot_behavior/navigation_inside_the_field.cpp
    robot_behavior/tutorials/beginner/goto_ball.cpp
    robot_behavior/tutorials/beginner/goalie.cpp
    robot_behavior/tutorials/beginner/go_corner.cpp
    robot_behavior/tutorials/beginner/robot_near_ball.cpp
    robot_behavior/tutorials/beginner/robot_have_ball.cpp
    robot_behavior/tutorials/beginner/annotations_ball_position.cpp
    robot_behavior/tutorials/beginner/see_ball.cpp
    robot_behavior/tutorials/beginner/see_robot.cpp
    robot_behavior/tutorials/medium/defender.cpp
    robot_behavior/keeper/keeper.cpp
    robot_behavior/defender/defensive_wall.cpp
    robot_behavior/defender/kick_wall.cpp
    robot_behavior/obstructor.cpp
    robot_behavior/striker.cpp
    ai.cpp
    debug.cpp
    strategy/strategy.cpp
    strategy/halt.cpp
    strategy/placer.cpp
    strategy/tare_and_synchronize.cpp
    strategy/from_robot_behavior.cpp
    strategy/keeper/keeper_strat.cpp
    control/robot_control.cpp
    control/robot_control_with_curve.cpp
    control/robot_control_with_position_following.cpp
    control/pid.cpp
    control/control.cpp
    control/kinematic.cpp
    physic/movement_predicted_by_integration.cpp
    physic/movement_with_no_prediction.cpp
    physic/movement_on_new_frame.cpp
    physic/movement_with_temporal_shift.cpp
    physic/movement_sample.cpp
    physic/movement.cpp
    physic/factory.cpp
    physic/collision.cpp
    math/continuous_angle.cpp
    math/curve.cpp
    math/tangents.cpp
    math/vector2d.cpp
    math/position.cpp
    math/matrix2d.cpp
    math/frame_changement.cpp
    math/intersection.cpp
    math/box.cpp
    math/lines.cpp
    core/export_to_plot.cpp
    core/logger.cpp
    core/gnu_plot.cpp
    referee/referee_packet_analyzer.cpp
    referee/game_state.cpp
    # Manager
    manager/manager.cpp
    manager/manager_with_game_state.cpp
    manager/manual.cpp
    manager/factory.cpp
    manager/match.cpp
    # Strategy
    strategy/wall.cpp
    strategy/wall_2.cpp
    # Behavior
    robot_behavior/keeper/clearer.cpp
    robot_behavior/attacker/receiver.cpp
    # Annotations
    annotations/annotations.cpp
    annotations/shape/shape.cpp
    annotations/shape/circle.cpp
    annotations/shape/cross.cpp
    annotations/shape/arrow.cpp
    annotations/shape/polygon.cpp
)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/../../bin/)
set(TEST_DIRECTORY ${CMAKE_SOURCE_DIR}/../../bin/)

set(ALL_LIBS ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} websockets)

add_library (ssl_ai ${SOURCES})
target_link_libraries(ssl_ai ${ALL_LIBS})

add_executable(ai main.cpp)
target_link_libraries(ai ssl_ai ${ALL_LIBS})

add_executable(see_robot executables/see_robot.cpp)
target_link_libraries(see_robot ssl_ai ${ALL_LIBS})

message(WARNING "CATKIN ENABLE TESTING: ${CATKIN_ENABLE_TESTING}")

if(CATKIN_ENABLE_TESTING)
  include(CTest)

  set (TEST_SOURCES
    core/test_machine_state.cpp
    core/test_collection.cpp
    core/test_print_collection.cpp
    control/test_control.cpp
    math/test_curve.cpp
    math/test_circular_vector.cpp
    math/test_frame_changement.cpp
    math/test_matching.cpp
    math/test_lines.cpp
    physic/test_movement_sample.cpp
    physic/test_movement_with_no_prediction.cpp
    physic/test_movement_predicted_by_integration.cpp
    physic/test_collision.cpp
    math/test_continuous_angle.cpp
    math/test_tangents.cpp
    math/test_vector2d.cpp
    math/test_matrix2d.cpp
  )

  foreach(test_source ${TEST_SOURCES})
      get_filename_component(name_test ${test_source} NAME_WE)
      catkin_add_gtest(${name_test} ${test_source})
      target_link_libraries(${name_test} ssl_ai ${ALL_LIBS})
  endforeach(test_source)
endif(CATKIN_ENABLE_TESTING)
