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

#include "caterpillar.h"

namespace rhoban_ssl
{
namespace strategy
{
Caterpillar::Caterpillar()
  : Strategy()
  , head_ball_(std::shared_ptr<robot_behavior::Beginner::Goto_ball>(new robot_behavior::Beginner::Goto_ball()))
  , path_mode_(false)
{
}

Caterpillar::Caterpillar(std::vector<rhoban_geometry::Point> path)
  : Strategy()
  , head_path_(std::shared_ptr<robot_behavior::GoToXY>(new robot_behavior::GoToXY()))
  , path_mode_(true)
  , path_(path)
  , path_index_(0)
{
  head_path_->setReachRadius(0.1);
}

Caterpillar::~Caterpillar()
{
}

/*
 * We define the minimal number of robot in the field.
 * The goalkeeper is not counted.
 */
int Caterpillar::minRobots() const
{
  return CATERPILLAR_SIZE;
}

/*
 * We define the maximal number of robot in the field.
 * The goalkeeper is not counted.
 */
int Caterpillar::maxRobots() const
{
  return CATERPILLAR_SIZE;
}

GoalieNeed Caterpillar::needsGoalie() const
{
  return GoalieNeed::NO;
}

const std::string Caterpillar::name = "LA CHENILLE !";

void Caterpillar::start(double time)
{
  DEBUG("START CATERPILLAR");
  behaviors_are_assigned_ = false;
  for (int i = 1; i < CATERPILLAR_SIZE; i++)
  {
    followers_.push_back(
        std::shared_ptr<robot_behavior::medium::FollowRobot>(new robot_behavior::medium::FollowRobot()));
  }
  if (path_mode_)
  {
    head_path_->setPoint(path_.at(0));
    path_index_++;
  }
}
void Caterpillar::stop(double time)
{
  DEBUG("STOP CATERPILLAR");
}

void Caterpillar::update(double time)
{
  if (path_mode_)
  {
    if (head_path_->isReached())
    {
      head_path_->setPoint(path_.at(path_index_ % path_.size()));
      path_index_++;
    }
  }
}

void Caterpillar::assignBehaviorToRobots(
    std::function<void(int, std::shared_ptr<robot_behavior::RobotBehavior>)> assign_behavior, double time, double dt)
{
  // head:
  if (path_mode_)
  {  // path mode, head indefintly visits in order a list of points.
    assign_behavior(playerId(0), head_path_);
  }
  else
  {  // normal mode, head always follows the ball.
    assign_behavior(playerId(0), head_ball_);
  }

  // queue:
  for (int i = 0; i < CATERPILLAR_SIZE - 1; i++)
  {
    followers_.at(i)->setRobotIdToFollow(playerId(i));
    assign_behavior(playerId(i + 1), followers_.at(i));
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
Caterpillar::getStartingPositions(int number_of_avalaible_robots)
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
bool Caterpillar::getStartingPositionForGoalie(rhoban_geometry::Point& linear_position,
                                               ContinuousAngle& angular_position)
{
  linear_position = Data::get()->field.goalCenter(Ally);
  angular_position = ContinuousAngle(0.0);
  return true;
}

rhoban_ssl::annotations::Annotations Caterpillar::getAnnotations() const
{
  rhoban_ssl::annotations::Annotations annotations;

  return annotations;
}

}  // namespace strategy
}  // namespace rhoban_ssl