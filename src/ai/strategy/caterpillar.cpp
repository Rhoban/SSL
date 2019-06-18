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
  , head_(std::shared_ptr<robot_behavior::Beginner::Goto_ball>(new robot_behavior::Beginner::Goto_ball()))
  , follower1_(std::shared_ptr<robot_behavior::medium::FollowRobot>(new robot_behavior::medium::FollowRobot()))
  , follower2_(std::shared_ptr<robot_behavior::medium::FollowRobot>(new robot_behavior::medium::FollowRobot()))
  , follower3_(std::shared_ptr<robot_behavior::medium::FollowRobot>(new robot_behavior::medium::FollowRobot()))
  , follower4_(std::shared_ptr<robot_behavior::medium::FollowRobot>(new robot_behavior::medium::FollowRobot()))
  , follower5_(std::shared_ptr<robot_behavior::medium::FollowRobot>(new robot_behavior::medium::FollowRobot()))
  , follower6_(std::shared_ptr<robot_behavior::medium::FollowRobot>(new robot_behavior::medium::FollowRobot()))
  , follower7_(std::shared_ptr<robot_behavior::medium::FollowRobot>(new robot_behavior::medium::FollowRobot()))
{
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
  return 8;
}

/*
 * We define the maximal number of robot in the field.
 * The goalkeeper is not counted.
 */
int Caterpillar::maxRobots() const
{
  return 8;
}

GoalieNeed Caterpillar::needsGoalie() const
{
  return GoalieNeed::NO;
}

const std::string Caterpillar::name = "LA CHENILLE !";

void Caterpillar::start(double time)
{
  DEBUG("START PREPARE KICKOFF");
  behaviors_are_assigned_ = false;
}
void Caterpillar::stop(double time)
{
  DEBUG("STOP PREPARE KICKOFF");
}

void Caterpillar::update(double time)
{
}

void Caterpillar::assignBehaviorToRobots(
    std::function<void(int, std::shared_ptr<robot_behavior::RobotBehavior>)> assign_behavior, double time, double dt)
{
  // head:
  assign_behavior(playerId(0), head_);

  // queue:
  follower1_->setRobotIdToFollow(playerId(0));
  assign_behavior(playerId(1), follower1_);
  follower2_->setRobotIdToFollow(playerId(1));
  assign_behavior(playerId(2), follower2_);
  follower3_->setRobotIdToFollow(playerId(2));
  assign_behavior(playerId(3), follower3_);
  follower4_->setRobotIdToFollow(playerId(3));
  assign_behavior(playerId(4), follower4_);
  follower5_->setRobotIdToFollow(playerId(4));
  assign_behavior(playerId(5), follower5_);
  follower6_->setRobotIdToFollow(playerId(5));
  assign_behavior(playerId(6), follower6_);
  follower7_->setRobotIdToFollow(playerId(6));
  assign_behavior(playerId(7), follower7_);

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

  for (auto it = this->getPlayerIds().begin(); it != this->getPlayerIds().end(); it++)
  {
    const rhoban_geometry::Point& robot_position = getRobot(*it).getMovement().linearPosition(time());
    // annotations.addText("Behaviour: " + this->name, robot_position.getX() + 0.15, robot_position.getY(), "white");
    annotations.addText("Strategy: " + this->name, robot_position.getX() + 0.15, robot_position.getY() + 0.30, "white");
  }
  return annotations;
}

}  // namespace strategy
}  // namespace rhoban_ssl