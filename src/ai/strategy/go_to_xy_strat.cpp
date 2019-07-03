/*
    This file is part of SSL.

    Copyright 2018 RomainPC (romainpc.lechat@laposte.net)

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

#include "go_to_xy_strat.h"

namespace rhoban_ssl
{
namespace strategy
{
GoToXYStrat::GoToXYStrat() : Strategy()
{
}

GoToXYStrat::GoToXYStrat(std::vector<rhoban_geometry::Point> positions) : Strategy(), positions_(positions)
{
  NB_ROBOT = positions.size();
  DEBUG("NB_ROBOT " << NB_ROBOT);
}

GoToXYStrat::~GoToXYStrat()
{
}

/*
 * We define the minimal number of robot in the field.
 * The goalkeeper is not counted.
 */
int GoToXYStrat::minRobots() const
{
  return NB_ROBOT;
}

/*
 * We define the maximal number of robot in the field.
 * The goalkeeper is not counted.
 */
int GoToXYStrat::maxRobots() const
{
  return NB_ROBOT;
}

GoalieNeed GoToXYStrat::needsGoalie() const
{
  return GoalieNeed::NO;
}

const std::string GoToXYStrat::name = "LA CHENILLE !";

void GoToXYStrat::start(double time)
{
  DEBUG("START GoToXYStrat");
  behaviors_are_assigned_ = false;
  for (int i = 0; i < NB_ROBOT; i++)
  {
    vect_go_to_xy_.push_back(
        std::shared_ptr<robot_behavior::GoToXY>(new robot_behavior::GoToXY()));
  }
}
void GoToXYStrat::stop(double time)
{
  DEBUG("STOP GoToXYStrat");
}

void GoToXYStrat::update(double time)
{
  
}

void GoToXYStrat::assignBehaviorToRobots(
    std::function<void(int, std::shared_ptr<robot_behavior::RobotBehavior>)> assign_behavior, double time, double dt)
{  // queue:
  for (int i = 0; i < NB_ROBOT ; i++)
  {
    static_cast<robot_behavior::GoToXY*>(vect_go_to_xy_[i].get())->setPoint(positions_[i]);
    assign_behavior(playerId(i), vect_go_to_xy_.at(i));
  }

  behaviors_are_assigned_ = true;
}

// We declare here the starting positions that are used to :
//   - place the robot during STOP referee state
//   - to compute the robot order of get_player_ids(),
//     we minimize the distance between
//     the startings points and all the robot position, just
//     before the start() or during the STOP referee state.
std::list<std::pair<rhoban_geometry::Point, ContinuousAngle> >
GoToXYStrat::getStartingPositions(int number_of_avalaible_robots)
{
  assert(minRobots() <= number_of_avalaible_robots);
  assert(maxRobots() == -1 or number_of_avalaible_robots <= maxRobots());

  return { std::pair<rhoban_geometry::Point, ContinuousAngle>(Data::get()->field.goalCenter(Ally), 0.0) };
}

//
// This function return false if you don't want to
// give a staring position. So the manager will chose
// a default position for you.
//
bool GoToXYStrat::getStartingPositionForGoalie(rhoban_geometry::Point& linear_position,
                                               ContinuousAngle& angular_position)
{
  linear_position = Data::get()->field.goalCenter(Ally);
  angular_position = ContinuousAngle(0.0);
  return true;
}

void GoToXYStrat::setPositions(std::vector<rhoban_geometry::Point> positions){
  positions_ = positions;
}

rhoban_ssl::annotations::Annotations GoToXYStrat::getAnnotations() const
{
  rhoban_ssl::annotations::Annotations annotations;

  return annotations;
}

}  // namespace strategy
}  // namespace rhoban_ssl