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

#include "defensive.h"

namespace rhoban_ssl
{
namespace Strategy
{
Defensive::Defensive(Ai::AiData& ai_data)
  : Strategy(ai_data)
  , degageur(std::shared_ptr<Robot_behavior::Degageur>(new Robot_behavior::Degageur(ai_data)))
  , obstructeur(std::shared_ptr<Robot_behavior::Obstructor>(new Robot_behavior::Obstructor(ai_data)))
{
}

Defensive::~Defensive()
{
}

/*
 * We define the minimal number of robot in the field.
 * The goalkeeper is not counted.
 */
int Defensive::min_robots() const
{
  return 1;
}

/*
 * We define the maximal number of robot in the field.
 * The goalkeeper is not counted.
 */
int Defensive::max_robots() const
{
  return 1;
}

Goalie_need Defensive::needs_goalie() const
{
  return Goalie_need::NO;
}

const std::string Defensive::name = "defensive";

void Defensive::start(double time)
{
  DEBUG("START PREPARE KICKOFF");
  behaviors_are_assigned = false;
}
void Defensive::stop(double time)
{
  DEBUG("STOP PREPARE KICKOFF");
}

void Defensive::update(double time)
{
}

void Defensive::assign_behavior_to_robots(
    std::function<void(int, std::shared_ptr<Robot_behavior::RobotBehavior>)> assign_behavior, double time, double dt)
{
  // we assign now all the other behavior
  assert(get_player_ids().size() == 1);
  int robotID = player_id(0);  // we get the first if in get_player_ids()

  int nearest_ballID = get_shirt_number_of_closest_robot_to_the_ball(Vision::Team::Ally);

  int id_to_obstruct = shirt_number_of_threat_max(Vision::Team::Opponent);
  obstructeur->declare_robot_to_obstruct(id_to_obstruct, Vision::Team::Opponent);
  if (nearest_ballID == robotID)
  {
    assign_behavior(robotID, degageur);
  }
  else
  {
    assign_behavior(robotID, obstructeur);
  }

  behaviors_are_assigned = true;
}

// We declare here the starting positions that are used to :
//   - place the robot during STOP referee state
//   - to compute the robot order of get_player_ids(),
//     we minimize the distance between
//     the startings points and all the robot position, just
//     before the start() or during the STOP referee state.
std::list<std::pair<rhoban_geometry::Point, ContinuousAngle> >
Defensive::get_starting_positions(int number_of_avalaible_robots)
{
  assert(min_robots() <= number_of_avalaible_robots);
  assert(max_robots() == -1 or number_of_avalaible_robots <= max_robots());

  return { std::pair<rhoban_geometry::Point, ContinuousAngle>(ally_goal_center(), 0.0) };
}

//
// This function return false if you don't want to
// give a staring position. So the manager will chose
// a default position for you.
//
bool Defensive::get_starting_position_for_goalie(rhoban_geometry::Point& linear_position,
                                                 ContinuousAngle& angular_position)
{
  linear_position = ally_goal_center();
  angular_position = ContinuousAngle(0.0);
  return true;
}

RhobanSSLAnnotation::Annotations Defensive::get_annotations() const
{
  RhobanSSLAnnotation::Annotations annotations;

  for (auto it = this->get_player_ids().begin(); it != this->get_player_ids().end(); it++)
  {
    const rhoban_geometry::Point& robot_position = get_robot(*it).get_movement().linear_position(time());
    // annotations.addText("Behaviour: " + this->name, robot_position.getX() + 0.15, robot_position.getY(), "white");
    annotations.addText("Strategy: " + this->name, robot_position.getX() + 0.15, robot_position.getY() + 0.30, "white");
  }
  return annotations;
}

}  // namespace Strategy
}  // namespace rhoban_ssl
