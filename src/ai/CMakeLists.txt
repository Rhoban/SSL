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
    data.cpp
    config.cpp
    viewer.cpp
    game_informations.cpp
    vision/ai_vision_client.cpp
    vision/vision_data.cpp
    vision/print_protobuf.cpp
    vision/robot_position_filter.cpp
    vision/factory.cpp
    com/ai_commander.cpp
    com/ai_commander_simulation.cpp
    com/ai_commander_real.cpp
    robot_behavior/factory.cpp
    robot_behavior/example_machine_state.cpp
    robot_behavior/example.cpp
    robot_behavior/goalie.cpp
    robot_behavior/patrol.cpp
    robot_behavior/pass.cpp
    robot_behavior/pass_dribbler.cpp
    robot_behavior/protect_ball.cpp
    robot_behavior/search_shoot_area.cpp
    robot_behavior/defensor.cpp
    robot_behavior/passive_defensor.cpp
    robot_behavior/mur_defensor.cpp
    robot_behavior/mur_def_kick.cpp
    robot_behavior/robot_follower.cpp
    robot_behavior/concept_proof_spinner.cpp
    robot_behavior/striker.cpp
    robot_behavior/striker_ai.cpp
    robot_behavior/predict_futur.cpp
    robot_behavior/degageur.cpp
    robot_behavior/test_kicker.cpp
    robot_behavior/test_velocity_consign.cpp
    robot_behavior/position_follower.cpp
    robot_behavior/do_nothing.cpp
    robot_behavior/a_star_path.cpp
    robot_behavior/consign_follower.cpp
    robot_behavior/robot_behavior.cpp
    robot_behavior/navigation_with_obstacle_avoidance.cpp
    robot_behavior/navigation_inside_the_field.cpp
    robot_behavior/obstructor.cpp
    robot_behavior/wait_pass.cpp
    robot_behavior/slow_striker.cpp
    robot_behavior/test_infra.cpp
    robot_behavior/tutorials/beginner/goto_ball.cpp
    robot_behavior/tutorials/beginner/goalie.cpp
    robot_behavior/tutorials/beginner/go_corner.cpp
    robot_behavior/tutorials/beginner/robot_near_ball.cpp
    robot_behavior/tutorials/beginner/robot_have_ball.cpp
    robot_behavior/tutorials/beginner/annotations_ball_position.cpp
    robot_behavior/tutorials/beginner/see_ball.cpp
    robot_behavior/tutorials/beginner/see_robot.cpp
    robot_behavior/tutorials/medium/prepare_strike.cpp
    robot_behavior/tutorials/medium/striker.cpp
    robot_behavior/tutorials/medium/defender.cpp
    robot_behavior/test_relative_velocity_consign.cpp
    ai.cpp
    ai_data.cpp
    debug.cpp
    manager/manager.cpp
    manager/manager_with_game_state.cpp
    manager/plan_veschambres.cpp
    # manager/Match.cpp
    manager/manual.cpp
    manager/factory.cpp
    strategy/strategy.cpp
    strategy/halt.cpp
    strategy/mur.cpp
    strategy/striker_with_support.cpp
    strategy/striker_v2.cpp
    strategy/base.cpp
    strategy/base_3_gds.cpp
    strategy/base_3_gms.cpp
    strategy/indirect.cpp
    strategy/indirect_lob.cpp
    strategy/attaque_with_support.cpp
    strategy/attaque_with_support_ms.cpp
    strategy/striker_kick.cpp
    strategy/defensive.cpp
    strategy/defensive_2.cpp
    strategy/union.cpp
    strategy/offensive.cpp
    strategy/tare_and_synchronize.cpp
    strategy/from_robot_behavior.cpp
    strategy/placer.cpp
    strategy/mur_2.cpp
    strategy/mur_2_passif.cpp
    strategy/mur_stop.cpp
    strategy/goalie_strat.cpp
    strategy/prepare_kickoff.cpp
    strategy/example.cpp
    control/robot_control.cpp
    control/robot_control_with_curve.cpp
    control/robot_control_with_position_following.cpp
    control/pid.cpp
    control/control.cpp
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
    referee/print_protobuf_referee.cpp
    referee/game_state.cpp
    annotations/annotations.cpp
)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/../../bin/)
set(TEST_DIRECTORY ${CMAKE_SOURCE_DIR}/../../bin/)

set(ALL_LIBS ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} zmq)

add_library (ssl_ai ${SOURCES})
target_link_libraries(ssl_ai ${ALL_LIBS})

add_executable(ai main.cpp)
target_link_libraries(ai ssl_ai ${ALL_LIBS})

add_executable(ai_st main_st.cpp)
target_link_libraries(ai_st ssl_ai ${ALL_LIBS})

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
    math/test_lines.cpp
    physic/test_movement_sample.cpp
    physic/test_movement_with_no_prediction.cpp
    physic/test_movement_predicted_by_integration.cpp
    physic/test_collision.cpp
    math/test_ContinuousAngle.cpp
    math/test_tangents.cpp
    math/test_vector2d.cpp
    math/test_matrix2d.cpp
    robot_behavior/test_robot_behavior.cpp
  )

  foreach(test_source ${TEST_SOURCES})
      get_filename_component(name_test ${test_source} NAME_WE)
      catkin_add_gtest(${name_test} ${test_source})
      target_link_libraries(${name_test} ssl_ai ${ALL_LIBS})
  endforeach(test_source)
endif(CATKIN_ENABLE_TESTING)
