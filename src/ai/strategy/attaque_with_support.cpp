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

#include "attaque_with_support.h"

namespace rhoban_ssl
{
namespace strategy
{
AttaqueWithSupport::AttaqueWithSupport(ai::AiData& ai_data)
  : Strategy(ai_data)
  , support_(std::shared_ptr<Robot_behavior::RobotFollower>(new Robot_behavior::RobotFollower(ai_data)))
  , pass_(std::shared_ptr<Robot_behavior::Pass>(new Robot_behavior::Pass(ai_data)))

{
}

AttaqueWithSupport::~AttaqueWithSupport()
{
}

/*
 * We define the minimal number of robot in the field.
 * The goalkeeper is not counted.
 */
int AttaqueWithSupport::minRobots() const
{
  return 2;
}

/*
 * We define the maximal number of robot in the field.
 * The goalkeeper is not counted.
 */
int AttaqueWithSupport::maxRobots() const
{
  return 2;
}

GoalieNeed AttaqueWithSupport::needsGoalie() const
{
  return GoalieNeed::NO;
}

const std::string AttaqueWithSupport::name = "attaque_with_support";

void AttaqueWithSupport::start(double time)
{
  DEBUG("START PREPARE KICKOFF");
  behaviors_are_assigned_ = false;

  striker_ = std::shared_ptr<Robot_behavior::Striker>(new Robot_behavior::Striker(ai_data_));
}
void AttaqueWithSupport::stop(double time)
{
  DEBUG("STOP PREPARE KICKOFF");
}

void AttaqueWithSupport::update(double time)
{
  results_ = GameInformations::findGoalBestMove(ballPosition());
  striker_->declare_point_to_strik(results_.first);
}

void AttaqueWithSupport::assignBehaviorToRobots(
    std::function<void(int, std::shared_ptr<Robot_behavior::RobotBehavior>)> assign_behavior, double time, double dt)
{
  if (not(behaviors_are_assigned_))
  {
    // we assign now all the other behavior
    assert(getPlayerIds().size() == 2);
    int ID1 = playerId(0);
    int ID2 = playerId(1);  // we get the first if in get_player_ids()

    const rhoban_geometry::Point& robot_position_1 =
        getRobot(ID1, vision::Team::Ally).getMovement().linearPosition(time);
    const rhoban_geometry::Point& robot_position_2 =
        getRobot(ID2, vision::Team::Ally).getMovement().linearPosition(time);

    double ball_robot1 = (Vector2d(robot_position_1 - ballPosition())).norm();
    double ball_robot2 = (Vector2d(robot_position_2 - ballPosition())).norm();

    int strikerID;
    int supportID;
    double seuil = 0.35;

    if (ball_robot1 < ball_robot2)
    {
      strikerID = ID1;
      supportID = ID2;
    }
    else
    {
      strikerID = ID2;
      supportID = ID1;
    }

    if (results_.second > seuil)
    {
      assign_behavior(strikerID, striker_);
      support_->declare_robot_to_follow(strikerID, Vector2d(1, 2), vision::Team::Ally);
      assign_behavior(supportID, support_);
    }
    else
    {
      pass_->declare_robot_to_pass(supportID, vision::Team::Ally);
      assign_behavior(strikerID, pass_);
    }
    support_->declare_robot_to_follow(strikerID, Vector2d(1, 2), vision::Team::Ally);
    assign_behavior(supportID, support_);

    // behaviors_are_assigned = true;
  }
}

// We declare here the starting positions that are used to :
//   - place the robot during STOP referee state
//   - to compute the robot order of get_player_ids(),
//     we minimize the distance between
//     the startings points and all the robot position, just
//     before the start() or during the STOP referee state.
std::list<std::pair<rhoban_geometry::Point, ContinuousAngle> >
AttaqueWithSupport::getStartingPositions(int number_of_avalaible_robots)
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
bool AttaqueWithSupport::getStartingPositionForGoalie(rhoban_geometry::Point& linear_position,
                                                          ContinuousAngle& angular_position)
{
  linear_position = allyGoalCenter();
  angular_position = ContinuousAngle(0.0);
  return true;
}

rhoban_ssl::annotations::Annotations AttaqueWithSupport::getAnnotations() const
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

}  // namespace Strategy
}  // namespace rhoban_ssl
