/*
This file is part of SSL.

Copyright 2018 Bezamat Jérémy (jeremy.bezamat@gmail.com)

SSL is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

SSL is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with SSL.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "attaque_with_support_ms.h"
#include "core/print_collection.h"


namespace RhobanSSL {
  namespace Strategy {

    AttaqueWithSupportMs::AttaqueWithSupportMs(Ai::AiData & ai_data):
      Strategy(ai_data),
      machine(ai_data, ai_data),
      striker_behavior(std::shared_ptr<Robot_behavior::StrikerAi>(
        new Robot_behavior::StrikerAi(ai_data)
      )),
      search_behavior(std::shared_ptr<Robot_behavior::SearchShootArea>(
        new Robot_behavior::SearchShootArea(ai_data)
      )),
      // pass_behavior(std::shared_ptr<Robot_behavior::Pass>(
      //   new Robot_behavior::Pass(ai_data)
      // )),
      pass_behavior(std::shared_ptr<Robot_behavior::Pass_dribbler>(
        new Robot_behavior::Pass_dribbler(ai_data)
      )),
      wait_pass_behavior(std::shared_ptr<Robot_behavior::WaitPass>(
        new Robot_behavior::WaitPass(ai_data)
      )),
      seuil_fgbm(0.2),
      tempo(3),
      begin_time(0)
    {

      //STATES
      machine.add_state(
        state_name::strike_search,
        [this]( const Ai::AiData & data, unsigned int run_number, unsigned int atomic_run_number ){
          DEBUG(state_name::strike_search);
        }
      );
      machine.add_state(
        state_name::search_strike,
        [this]( const Ai::AiData & data, unsigned int run_number, unsigned int atomic_run_number ){
          DEBUG(state_name::search_strike);
        }
      );

      machine.add_state(
        state_name::pass_search,
        [this]( const Ai::AiData & data, unsigned int run_number, unsigned int atomic_run_number ){
          DEBUG(state_name::pass_search);
        }
      );
      machine.add_state(
        state_name::search_pass,
        [this]( const Ai::AiData & data, unsigned int run_number, unsigned int atomic_run_number ){
          DEBUG(state_name::search_pass);
        }
      );

      machine.add_state(
        state_name::search_waitpass,
        [this]( const Ai::AiData & data, unsigned int run_number, unsigned int atomic_run_number ){
          DEBUG(state_name::search_waitpass);
        }
      );
      machine.add_state(
        state_name::waitpass_search,
        [this]( const Ai::AiData & data, unsigned int run_number, unsigned int atomic_run_number ){
          DEBUG(state_name::waitpass_search);
        }
      );


      //EDGES
      machine.add_edge(
        edge_name::db1_sup_db2,
        state_name::strike_search, state_name::search_strike,
        [this](const Ai::AiData & data, unsigned int run_number, unsigned int atomic_run_number ){
          return is_db1_sup_db2();
        }
      );
      machine.add_edge(
        edge_name::db1_inf_db2,
        state_name::search_strike, state_name::strike_search,
        [this](const Ai::AiData & data, unsigned int run_number, unsigned int atomic_run_number ){
          return is_db1_inf_db2();
        }
      );

      machine.add_edge(
        edge_name::fgbm_score_inf_seuil_1,
        state_name::strike_search, state_name::pass_search,
        [this](const Ai::AiData & data, unsigned int run_number, unsigned int atomic_run_number ){
          return is_fgbm_score_inf_seuil_1();
        }
      );
      machine.add_edge(
        edge_name::fgbm_score_inf_seuil_2,
        state_name::search_strike, state_name::search_pass,
        [this](const Ai::AiData & data, unsigned int run_number, unsigned int atomic_run_number ){
          return is_fgbm_score_inf_seuil_2();
        }
      );

      machine.add_edge(
        edge_name::infra_1_on,
        state_name::pass_search, state_name::search_waitpass,
        [this](const Ai::AiData & data, unsigned int run_number, unsigned int atomic_run_number ){
          begin_time = time();
          return is_infra_1_on();
        }
      );
      machine.add_edge(
        edge_name::infra_2_on,
        state_name::search_pass, state_name::waitpass_search,
        [this](const Ai::AiData & data, unsigned int run_number, unsigned int atomic_run_number ){
          begin_time = time();
          return is_infra_2_on();
        }
      );

      machine.add_edge(
        edge_name::db2_inf_seuil_or_time_inf_tempo,
        state_name::search_waitpass, state_name::search_strike,
        [this](const Ai::AiData & data, unsigned int run_number, unsigned int atomic_run_number ){
          return is_db2_inf_seuil_or_time_inf_tempo();
        }
      );
      machine.add_edge(
        edge_name::db1_inf_seuil_or_time_inf_tempo,
        state_name::waitpass_search, state_name::strike_search,
        [this](const Ai::AiData & data, unsigned int run_number, unsigned int atomic_run_number ){
          return is_db1_inf_seuil_or_time_inf_tempo();
        }
      );


      machine.export_to_file("/tmp/attaque_with_support_ms.dot");

    }

    AttaqueWithSupportMs::~AttaqueWithSupportMs(){
    }

    /*
    * We define the minimal number of robot in the field.
    * The goalkeeper is not counted.
    */
    int AttaqueWithSupportMs::min_robots() const {
      return 2;
    }

    /*
    * We define the maximal number of robot in the field.
    * The goalkeeper is not counted.
    */
    int AttaqueWithSupportMs::max_robots() const {
      return 2;
    }

    Goalie_need AttaqueWithSupportMs::needs_goalie() const {
      return Goalie_need::NO;
    }

    const std::string AttaqueWithSupportMs::name = "attaque_with_support_ms";

    void AttaqueWithSupportMs::start(double time){
      DEBUG("START PREPARE KICKOFF");

      ID1 = player_id(0);
      ID2 = player_id(1); // we get the first if in get_player_ids()

      robot_1_position =  get_robot(ID1, Vision::Team::Ally).get_movement().linear_position( time );
      robot_2_position =  get_robot(ID2, Vision::Team::Ally).get_movement().linear_position( time );

      double db1 = (Vector2d (ball_position() - robot_1_position)).norm();
      double db2 = (Vector2d (ball_position() - robot_2_position)).norm();

      if (db1 > db2) {
        machine.add_init_state( state_name::strike_search );
      }else{
        machine.add_init_state( state_name::search_strike );
      }
      machine.start();

      behaviors_are_assigned = false;
    }
    void AttaqueWithSupportMs::stop(double time){
      DEBUG("STOP PREPARE KICKOFF");
    }

    void AttaqueWithSupportMs::update(double time){
    }

    void AttaqueWithSupportMs::assign_behavior_to_robots(
      std::function<
      void (int, std::shared_ptr<Robot_behavior::RobotBehavior>)
      > assign_behavior,
      double time, double dt
    ){
      //we assign now all the other behavior
      assert( get_player_ids().size() == 2 );

      robot_1_position =  get_robot(ID1, Vision::Team::Ally).get_movement().linear_position( time );
      robot_2_position =  get_robot(ID2, Vision::Team::Ally).get_movement().linear_position( time );

      machine.run();

      std::string state = *machine.current_states().begin();
      // DEBUG(machine.current_states());
      if ( state == state_name::strike_search) {
        assign_behavior( ID1, striker_behavior );
        assign_behavior( ID2, search_behavior );
      }else if( state == state_name::search_strike ){
        assign_behavior( ID1, search_behavior );
        assign_behavior( ID2, striker_behavior );
      }else if( state == state_name::pass_search ){
        assign_behavior( ID1, pass_behavior );
        pass_behavior->declare_robot_to_pass( ID2, Vision::Team::Ally );
        assign_behavior( ID2, search_behavior );
      }else if( state == state_name::search_pass ){
        assign_behavior( ID1, search_behavior );
        assign_behavior( ID2, pass_behavior );
        pass_behavior->declare_robot_to_pass( ID1, Vision::Team::Ally );
      }else if( state == state_name::search_waitpass ){
        assign_behavior( ID1, search_behavior );//pass_behavior );
        pass_behavior->declare_robot_to_pass( ID2, Vision::Team::Ally );
        // assign_behavior( ID2, wait_pass_behavior );
      }else if( state == state_name::waitpass_search ){
        assign_behavior( ID1, wait_pass_behavior );
        assign_behavior( ID2, search_behavior );//pass_behavior );
        // pass_behavior->declare_robot_to_pass( ID1, Vision::Team::Ally );
      }

      // std::pair<rhoban_geometry::Point, double> p_best = find_goal_best_move( ball_position() );
      // double score = 0;// p_best.second;
      // double seuil = 0.2;
      // if ( score > seuil ) {
      //   // striker_behavior->declare_point_to_strik(p_best.first);
      //   assign_behavior( strikerID, striker_behavior );
      //   assign_behavior( supportID, search_behavior );
      // } else{
      //   pass_behavior->declare_robot_to_pass( supportID, Vision::Team::Ally );
      //   assign_behavior( strikerID, pass_behavior );
      // }
      // assign_behavior( supportID, search_behavior );


      // behaviors_are_assigned = true;

    }

    // We declare here the starting positions that are used to :
    //   - place the robot during STOP referee state
    //   - to compute the robot order of get_player_ids(),
    //     we minimize the distance between
    //     the startings points and all the robot position, just
    //     before the start() or during the STOP referee state.
    std::list<
    std::pair<rhoban_geometry::Point,ContinuousAngle>
    > AttaqueWithSupportMs::get_starting_positions( int number_of_avalaible_robots ){
      assert( min_robots() <= number_of_avalaible_robots );
      assert(
        max_robots()==-1 or
        number_of_avalaible_robots <= max_robots()
      );

      return {
        std::pair<rhoban_geometry::Point,ContinuousAngle>(
          ball_position(),
          0.0
        )
      };
    }

    //
    // This function return false if you don't want to
    // give a staring position. So the manager will chose
    // a default position for you.
    //
    bool AttaqueWithSupportMs::get_starting_position_for_goalie(
      rhoban_geometry::Point & linear_position,
      ContinuousAngle & angular_position
    ){
      linear_position =  ally_goal_center();
      angular_position = ContinuousAngle(0.0);
      return true;
    }


    bool AttaqueWithSupportMs::is_db1_sup_db2(){
      double db1 = (Vector2d (ball_position() - robot_1_position)).norm();
      double db2 = (Vector2d (ball_position() - robot_2_position)).norm();
      // DEBUG("is_db1_sup_db2 " << db1 << " " << db2);
      double fgbm_score = find_goal_best_move( ball_position() ).second;
      return ((db1 >= db2) and (fgbm_score >= seuil_fgbm));
    }
    bool AttaqueWithSupportMs::is_db1_inf_db2(){
      double db1 = (Vector2d (ball_position() - robot_1_position)).norm();
      double db2 = (Vector2d (ball_position() - robot_2_position)).norm();
      // DEBUG("is_db1_inf_db2 " << db1 << " " << db2);
      double fgbm_score = find_goal_best_move( ball_position() ).second;
      return ((db1 < db2) and (fgbm_score >= seuil_fgbm));
    }

    bool AttaqueWithSupportMs::is_fgbm_score_inf_seuil_1(){
      double fgbm_score = find_goal_best_move( ball_position() ).second;
      return (fgbm_score < seuil_fgbm);
    }
    bool AttaqueWithSupportMs::is_fgbm_score_inf_seuil_2(){
      double fgbm_score = find_goal_best_move( ball_position() ).second;
      return (fgbm_score < seuil_fgbm);
    }

    bool AttaqueWithSupportMs::is_infra_1_on(){
      return infra_red( ID1, Vision::Team::Ally);
      // double db1 = (Vector2d (ball_position() - robot_1_position)).norm();
      // // DEBUG("DB1 " << db1 );
      // return (db1 < get_robot_radius()+0.1);
    }
    bool AttaqueWithSupportMs::is_infra_2_on(){
      return infra_red( ID2, Vision::Team::Ally);
      // double db2 = (Vector2d (ball_position() - robot_2_position)).norm();
      // // DEBUG("DB2 " << db2 );
      // return (db2 < get_robot_radius()+0.1);
    }

    bool AttaqueWithSupportMs::is_db1_inf_seuil_or_time_inf_tempo(){
      return infra_red( ID1, Vision::Team::Ally);
      // double db1 = (Vector2d (ball_position() - robot_1_position)).norm();
      // bool t = (time() - begin_time > tempo);
      // return ((db1 < get_robot_radius()+0.1) || t);
    }
    bool AttaqueWithSupportMs::is_db2_inf_seuil_or_time_inf_tempo(){
      return infra_red( ID2, Vision::Team::Ally);
      // double db2 = (Vector2d (ball_position() - robot_2_position)).norm();
      // bool t = (time() - begin_time > tempo);
      // return ((db2 < get_robot_radius()+0.1) || t);
    }

    void AttaqueWithSupportMs::set_seuil_fgbm(double seuil){
      seuil_fgbm = seuil;
    }

  }
}
