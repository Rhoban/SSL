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

#include "mur_stop.h"

//#include <robot_behavior/goalie.h>
//#include <robot_behavior/striker.h>
#include <robot_behavior/defender/kick_wall.h>
//#include <robot_behavior/degageur.h>

namespace rhoban_ssl
{
namespace strategy
{
MurStop::MurStop() : Strategy()
{
}

MurStop::~MurStop()
{
}

/*
 * We define the minimal number of robot in the field.
 * The goalkeeper is not counted.
 */
int MurStop::minRobots() const
{
  return 2;
}

/*
 * We define the maximal number of robot in the field.
 * The goalkeeper is not counted.
 */
int MurStop::maxRobots() const
{
  return 2;
}

GoalieNeed MurStop::needsGoalie() const
{
  return GoalieNeed::NO;
}

const std::string MurStop::name = "mur_stop";

void MurStop::start(double time)
{
  DEBUG("START PREPARE KICKOFF");
  behaviors_are_assigned_ = false;
}
void MurStop::stop(double time)
{
  DEBUG("STOP PREPARE KICKOFF");
}

void MurStop::update(double time)
{
}

void MurStop::assignBehaviorToRobots(
    std::function<void(int, std::shared_ptr<robot_behavior::RobotBehavior>)> assign_behavior, double time, double dt)
{
  std::shared_ptr<robot_behavior::RobotBehavior> mur1(new robot_behavior::KickWall(1));
  static_cast<robot_behavior::KickWall*>(mur1.get())->declareMurRobotId(0, 2);

  std::shared_ptr<robot_behavior::RobotBehavior> mur2(new robot_behavior::KickWall(1));
  static_cast<robot_behavior::KickWall*>(mur2.get())->declareMurRobotId(1, 2);

  if (not(behaviors_are_assigned_))
  {
    assert(getPlayerIds().size() == 2);

    assign_behavior(playerId(0), mur1);
    assign_behavior(playerId(1), mur2);

    behaviors_are_assigned_ = true;
  }
}

// We declare here the starting positions that are used to :
//   - place the robot during STOP referee state
//   - to compute the robot order of get_player_ids(),
//     we minimize the distance between
//     the startings points and all the robot position, just
//     before the start() or during the STOP referee state.
std::list<std::pair<rhoban_geometry::Point, ContinuousAngle> >
MurStop::getStartingPositions(int number_of_avalaible_robots)
{
  assert(minRobots() <= number_of_avalaible_robots);
  assert(maxRobots() == -1 or number_of_avalaible_robots <= maxRobots());

  return { std::pair<rhoban_geometry::Point, ContinuousAngle>(ballPosition(), 0.0) };
}

//
// This function return false if you don't want to
// give a staring position. So the manager will chose
// a default position for you.
//
bool MurStop::getStartingPositionForGoalie(rhoban_geometry::Point& linear_position, ContinuousAngle& angular_position)
{
  linear_position = Data::get()->field.goalCenter(Ally);
  angular_position = ContinuousAngle(0.0);
  return true;
}

rhoban_ssl::annotations::Annotations MurStop::getAnnotations() const
{
  rhoban_ssl::annotations::Annotations annotations;
  /*
    for (auto it = this->getPlayerIds().begin(); it != this->getPlayerIds().end(); it++)
    {
      const rhoban_geometry::Point& robot_position = getRobot(*it).getMovement().linearPosition(time());
      // annotations.addText("Behaviour: " + this->name, robot_position.getX() + 0.15, robot_position.getY(), "white");
      annotations.addText("Strategy: " + this->name, robot_position.getX() + 0.15, robot_position.getY() + 0.30,
    "white");
    }*/
  return annotations;
}

}  // namespace strategy
}  // namespace rhoban_ssl
