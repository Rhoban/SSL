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

#include "defensive_2.h"


namespace RhobanSSL {
namespace Strategy {

Defensive2::Defensive2(Ai::AiData & ai_data):
    Strategy(ai_data),
    degageur1(std::shared_ptr<Robot_behavior::Degageur>(
      new Robot_behavior::Degageur(ai_data)
    )),
    obstructeur1(std::shared_ptr<Robot_behavior::Obstructor>(
      new Robot_behavior::Obstructor(ai_data)
    )),
    degageur2(std::shared_ptr<Robot_behavior::Degageur>(
      new Robot_behavior::Degageur(ai_data)
    )),
    obstructeur2(std::shared_ptr<Robot_behavior::Obstructor>(
      new Robot_behavior::Obstructor(ai_data)
    ))
{
}

Defensive2::~Defensive2(){
}

/*
 * We define the minimal number of robot in the field.
 * The goalkeeper is not counted.
 */
int Defensive2::min_robots() const {
    return 2;
}

/*
 * We define the maximal number of robot in the field.
 * The goalkeeper is not counted.
 */
int Defensive2::max_robots() const {
    return 2;
}

Goalie_need Defensive2::needs_goalie() const {
    return Goalie_need::NO;
}

const std::string Defensive2::name = "defensive2";

void Defensive2::start(double time){
    DEBUG("START PREPARE KICKOFF");
    behaviors_are_assigned = false;
}
void Defensive2::stop(double time){
    DEBUG("STOP PREPARE KICKOFF");
}

void Defensive2::update(double time){
}

void Defensive2::assign_behavior_to_robots(
  std::function<
  void (int, std::shared_ptr<Robot_behavior::RobotBehavior>)
  > assign_behavior,
  double time, double dt
){
  //we assign now all the other behavior
  assert( get_player_ids().size() == 2 );

  int id_to_obstruct1 = number_of_threat_max( Vision::Team::Opponent );
  int id_to_obstruct2 = number_of_threat_max_2( Vision::Team::Opponent );
  int robotID1 = player_id(0);
  int robotID2 = player_id(1);

  const Ai::Robot & robot1 = get_robot(robotID1, Vision::Team::Ally);
  const Ai::Robot & robot2 = get_robot(robotID2, Vision::Team::Ally);
  const rhoban_geometry::Point & robot_position_1 = robot1.get_movement().linear_position( time );
  const rhoban_geometry::Point & robot_position_2 = robot2.get_movement().linear_position( time );

  const Ai::Robot & robot_to_obstruct1 = get_robot(id_to_obstruct1, Vision::Team::Opponent);
  const rhoban_geometry::Point & robot_to_obstruct_position1 = robot_to_obstruct1.get_movement().linear_position( time );

  double distance1 = (Vector2d(robot_position_1 - robot_to_obstruct_position1)).norm();
  double distance2 = (Vector2d(robot_position_2 - robot_to_obstruct_position1)).norm();

  if(distance1 < distance2){
    obstructeur1->declare_robot_to_obstruct(id_to_obstruct1, Vision::Team::Opponent);
    obstructeur2->declare_robot_to_obstruct(id_to_obstruct2, Vision::Team::Opponent);
  }else{
    obstructeur1->declare_robot_to_obstruct(id_to_obstruct2, Vision::Team::Opponent);
    obstructeur2->declare_robot_to_obstruct(id_to_obstruct1, Vision::Team::Opponent);
  }



  int nearest_ballID = get_nearest_ball( Vision::Team::Ally );

  if ( nearest_ballID == robotID1 ) {
    assign_behavior( robotID1, degageur1 );
  }
  else{
    assign_behavior( robotID1, obstructeur1 );
  }


  if ( nearest_ballID == robotID2 ) {
    assign_behavior( robotID2, degageur2 );
  }
  else{
    assign_behavior( robotID2, obstructeur2 );
  }

  behaviors_are_assigned = true;
}

// We declare here the starting positions that are used to :
//   - place the robot during STOP referee state
//   - to compute the robot order of get_player_ids(),
//     we minimize the distance between
//     the startings points and all the robot position, just
//     before the start() or during the STOP referee state.
std::list<
    std::pair<rhoban_geometry::Point,ContinuousAngle>
> Defensive2::get_starting_positions( int number_of_avalaible_robots ){
    assert( min_robots() <= number_of_avalaible_robots );
    assert(
        max_robots()==-1 or
        number_of_avalaible_robots <= max_robots()
    );

    return {
        std::pair<rhoban_geometry::Point,ContinuousAngle>(
            ally_goal_center(),
            0.0
        )
    };
}

//
// This function return false if you don't want to
// give a staring position. So the manager will chose
// a default position for you.
//
bool Defensive2::get_starting_position_for_goalie(
    rhoban_geometry::Point & linear_position,
    ContinuousAngle & angular_position
){
    linear_position =  ally_goal_center();
    angular_position = ContinuousAngle(0.0);
    return true;
}

RhobanSSLAnnotation::Annotations Defensive2::get_annotations() const {
    RhobanSSLAnnotation::Annotations annotations;

    for (auto it = this->get_player_ids().begin(); it != this->get_player_ids().end(); it++)
    {
        const rhoban_geometry::Point & robot_position = get_robot(*it).get_movement().linear_position( time() );
        //annotations.addText("Behaviour: " + this->name, robot_position.getX() + 0.15, robot_position.getY(), "white");
        annotations.addText("Strategy: " + this->name, robot_position.getX() + 0.15, robot_position.getY() + 0.30, "white");
    }
    return annotations;
}

}
}
