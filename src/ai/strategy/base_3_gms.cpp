/*
    This file is part of SSL.

    Copyright 2018 Boussicault Adrien (adrien.boussicault@u-bordeaux.fr)

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

#include "base_3_gms.h"

#include <robot_behavior/goalie.h>
#include <robot_behavior/striker.h>
#include <robot_behavior/mur_defensor.h>

namespace rhoban_ssl
{
namespace strategy
{
Base3Gms::Base3Gms(ai::AiData& ai_data) : Strategy(ai_data)
{
}

Base3Gms::~Base3Gms()
{
}

/*
 * We define the minimal number of robot in the field.
 * The goalkeeper is not counted.
 */
int Base3Gms::minRobots() const
{
  return 3;
}

/*
 * We define the maximal number of robot in the field.
 * The goalkeeper is not counted.
 */
int Base3Gms::maxRobots() const
{
  return 3;
}

GoalieNeed Base3Gms::needsGoalie() const
{
  return GoalieNeed::YES;
}

const std::string Base3Gms::name = "Base_3_gms";

void Base3Gms::start(double time)
{
  DEBUG("START PREPARE KICKOFF");
  behaviors_are_assigned = false;
}
void Base3Gms::stop(double time)
{
  DEBUG("STOP PREPARE KICKOFF");
}

void Base3Gms::update(double time)
{
}

void Base3Gms::assignBehaviorToRobots(
    std::function<void(int, std::shared_ptr<Robot_behavior::RobotBehavior>)> assign_behavior, double time, double dt)
{
  if (not(behaviors_are_assigned))
  {
    // We first assign the behhavior of the goalie.
    assign_behavior(getGoalie(), std::shared_ptr<Robot_behavior::RobotBehavior>(new Robot_behavior::Goalie(ai_data_)));

    // we assign now all the other behavior
    assert(getPlayerIds().size() == 3);

    assign_behavior(playerId(0), std::shared_ptr<Robot_behavior::RobotBehavior>(new Robot_behavior::Striker(ai_data_)));
    assign_behavior(playerId(1),
                    std::shared_ptr<Robot_behavior::RobotBehavior>(new Robot_behavior::Mur_defensor(ai_data_)));

    behaviors_are_assigned = true;
  }
}

// We declare here the starting positions that are used to :
//   - place the robot during STOP referee state
//   - to compute the robot order of get_player_ids(),
//     we minimize the distance between
//     the startings points and all the robot position, just
//     before the start() or during the STOP referee state.
std::list<std::pair<rhoban_geometry::Point, ContinuousAngle> >
Base3Gms::getStartingPositions(int number_of_avalaible_robots)
{
  assert(minRobots() <= number_of_avalaible_robots);
  assert(maxRobots() == -1 or number_of_avalaible_robots <= maxRobots());

  return { std::pair<rhoban_geometry::Point, ContinuousAngle>(ai_data_.relative2absolute(-1.0 / 3.0, 0.0), 0.0) };
}

//
// This function return false if you don't want to
// give a staring position. So the manager will chose
// a default position for you.
//
bool Base3Gms::getStartingPositionForGoalie(rhoban_geometry::Point& linear_position,
                                                  ContinuousAngle& angular_position)
{
  linear_position = allyGoalCenter();
  angular_position = ContinuousAngle(0.0);
  return true;
}

}  // namespace Strategy
}  // namespace rhoban_ssl
