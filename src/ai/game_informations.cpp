/*
    This file is part of SSL.

    Copyright 2018 Boussicault Adrien (adrien.boussicault@u-bordeaux.fr)
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

#include "game_informations.h"
#include "math/lines.h"

namespace rhoban_ssl
{
GameInformations::GameInformations(ai::AiData& ai_data) : ai_data(ai_data)
{
}

GameInformations::~GameInformations()
{
}

double GameInformations::time() const
{
  return ai_data.time;
}

rhoban_geometry::Point GameInformations::allyGoalCenter() const
{
  return rhoban_geometry::Point(-ai_data.field.fieldLength / 2.0, 0.0);
}

rhoban_geometry::Point GameInformations::opponentGoalCenter() const
{
  return rhoban_geometry::Point(ai_data.field.fieldLength / 2.0, 0.0);
}

rhoban_geometry::Point GameInformations::centerMark() const
{
  return rhoban_geometry::Point(0.0, 0.0);
}

rhoban_geometry::Point GameInformations::opponentCornerRight() const
{
  return rhoban_geometry::Point(ai_data.field.fieldLength / 2.0, -ai_data.field.fieldWidth / 2.0);
}

rhoban_geometry::Point GameInformations::opponentCornerLeft() const
{
  return rhoban_geometry::Point(ai_data.field.fieldLength / 2.0, ai_data.field.fieldWidth / 2.0);
}

const ai::Robot& GameInformations::getRobot(int robot_number, vision::Team team) const
{
  return ai_data.robots.at(team).at(robot_number);
}

void GameInformations::getRobotInLine(const rhoban_geometry::Point p1, const rhoban_geometry::Point p2,
                                         vision::Team team, double distance, std::vector<int>& result) const
{
  if (norm_square(p1 - p2) == 0)
  {
    return;
  }

  for (size_t i = 0; i < ai::Constants::NB_OF_ROBOTS_BY_TEAM; i++)
  {
    const ai::Robot& robot = getRobot(i, team);
    if (robot.isPresentInVision())
    {
      const rhoban_geometry::Point& robot_position = robot.getMovement().linear_position(time());
      if (distanceFromPointToLine(robot_position, p1, p2) <= distance)
      {
        result.push_back(i);
      }
    }
  }
}

std::vector<int> GameInformations::getRobotInLine(const rhoban_geometry::Point p1, const rhoban_geometry::Point p2,
                                                     vision::Team team, double distance) const
{
  std::vector<int> result;
  getRobotInLine(p1, p2, team, distance, result);
  return result;
}

std::vector<int> GameInformations::getRobotInLine(const rhoban_geometry::Point p1, const rhoban_geometry::Point p2,
                                                     double distance) const
{
  std::vector<int> result;
  getRobotInLine(p1, p2, vision::Team::Ally, distance, result);
  getRobotInLine(p1, p2, vision::Team::Opponent, distance, result);
  return result;
}

std::pair<rhoban_geometry::Point, double> GameInformations::findGoalBestMove(const rhoban_geometry::Point point,
                                                                                const rhoban_geometry::Point goal) const
{
  rhoban_geometry::Point opponent_goal_point;
  if (goal == rhoban_geometry::Point(66, 66))
  {
    opponent_goal_point = opponentGoalCenter();
  }
  else
  {
    opponent_goal_point = goal;
  }

  rhoban_geometry::Point return_point;
  const rhoban_geometry::Point left_post_position =
      rhoban_geometry::Point(ai_data.field.fieldLength / 2.0, ai_data.field.goalWidth / 2.0);
  const rhoban_geometry::Point right_post_position =
      rhoban_geometry::Point(ai_data.field.fieldLength / 2.0, -ai_data.field.goalWidth / 2.0);
  const Vector2d left_right_post_vector = right_post_position - left_post_position;
  const double dist_post = left_right_post_vector.norm();
  const int nb_analysed_point = 16;
  rhoban_geometry::Point analysed_point;

  int nb_valid_path = 0;
  double max_valid_path = 0.0;
  int max_i = 0;
  bool max_valid_combo_begin = false;

  for (size_t i = 1; i < nb_analysed_point - 1; i++)
  {
    analysed_point = right_post_position + rhoban_geometry::Point(0, dist_post / nb_analysed_point * i);
    std::vector<int> robot_in_line =
        GameInformations::getRobotInLine(point, analysed_point, vision::Team::Opponent, 0.15);
    std::vector<int> robot_in_line2 =
        GameInformations::getRobotInLine(point, analysed_point, vision::Team::Ally, 0.15);
    robot_in_line.insert(robot_in_line.end(), robot_in_line2.begin(), robot_in_line2.end());
    if (robot_in_line.empty())
    {
      nb_valid_path++;
      if (nb_valid_path > max_valid_path)
      {
        max_valid_path = nb_valid_path;
        if (max_valid_combo_begin == false)
        {
          max_valid_combo_begin = true;
          max_i = i;
        }
      }
    }
    else
    {
      nb_valid_path = 0;
      max_valid_combo_begin = false;
    }
  }

  if (max_valid_path == 0)
  {
    return_point = opponent_goal_point;
  }
  else
  {
    return_point =
        right_post_position + rhoban_geometry::Point(0, dist_post / nb_analysed_point * (max_i + max_valid_path / 2));
  }

  double proba = max_valid_path / nb_analysed_point;

  std::pair<rhoban_geometry::Point, double> results(return_point, proba);

  return results;
}

int GameInformations::getShirtNumberOfClosestRobotToTheBall(vision::Team team) const
{
  return getShirtNumberOfClosestRobot(team, ballPosition());
}

int GameInformations::getShirtNumberOfClosestRobot(vision::Team team, rhoban_geometry::Point point) const
{
  int id = -1;
  double distance_max = -1;
  for (int i = 0; i < ai::Constants::NB_OF_ROBOTS_BY_TEAM; i++)
  {
    const ai::Robot& robot = getRobot(i, team);
    if (robot.isPresentInVision())
    {
      const rhoban_geometry::Point& robot_position = robot.getMovement().linear_position(time());
      double distance = robot_position.getDist(point);
      if (id == -1 or distance < distance_max)
      {
        distance_max = distance;
        id = i;
      }
    }
  }
  return id;
}

double GameInformations::getRobotDistanceFromAllyGoalCenter(int robot_number, vision::Team team) const
{
  double distance = -1;
  const ai::Robot& robot = getRobot(robot_number, team);
  if (robot.isPresentInVision())
  {
    const rhoban_geometry::Point& robot_position = robot.getMovement().linear_position(time());
    Vector2d goal_center_robot = robot_position - allyGoalCenter();
    distance = goal_center_robot.norm();
    distance = (ai_data.field.fieldLength - distance) / ai_data.field.fieldLength;
  }
  return distance;
}

std::vector<double> GameInformations::threat(vision::Team team) const
{
  std::vector<double> v_threat;
  for (size_t i = 0; i < ai::Constants::NB_OF_ROBOTS_BY_TEAM; i++)
  {
    double threat = getRobotDistanceFromAllyGoalCenter(i, team);
    v_threat.push_back(threat);
  }
  return v_threat;
}

int GameInformations::shirtNumberOfThreatMax(vision::Team team) const
{
  int id = -1;
  double threat_max = -1;

  std::vector<double> v_threat = threat(team);
  for (size_t i = 0; i < v_threat.size(); i++)
  {
    double threat = v_threat[i];
    if (threat > threat_max)
    {
      threat_max = threat;
      id = i;
    }
  }
  return id;
}

int GameInformations::shirtNumberOfThreatMax2(vision::Team team) const
{  // second threat max
  int id_1 = -1;
  int id_2 = -1;
  double threat_max = -1;
  double threat_max_2 = -1;

  std::vector<double> v_threat = threat(team);
  for (size_t i = 0; i < v_threat.size(); i++)
  {
    double threat = v_threat[i];
    if (threat > threat_max)
    {
      threat_max_2 = threat_max;
      threat_max = threat;
      id_2 = id_1;
      id_1 = i;
    }
    else if (threat > threat_max_2)
    {
      threat_max_2 = threat;
      id_2 = i;
    }
  }
  return id_2;
}

const ai::Ball& GameInformations::ball() const
{
  return ai_data.ball;
}

rhoban_geometry::Point GameInformations::ballPosition() const
{
  return ball().getMovement().linear_position(time());
}

rhoban_geometry::Point GameInformations::centerAllyField() const
{
  return rhoban_geometry::Point(-ai_data.field.fieldLength / 4.0, 0.0);
}
rhoban_geometry::Point GameInformations::centerOpponentField() const
{
  return rhoban_geometry::Point(ai_data.field.fieldLength / 4.0, 0.0);
}

double GameInformations::getRobotRadius() const
{
  return ai_data.constants.robot_radius;
}

double GameInformations::getBallRadius() const
{
  return ai_data.constants.radius_ball;
}

std::vector<rhoban_geometry::Point> GameInformations::centerQuarterField() const
{
  return std::vector<rhoban_geometry::Point>({
      rhoban_geometry::Point(ai_data.field.fieldLength / 4.0, ai_data.field.fieldWidth / 4.0),
      rhoban_geometry::Point(ai_data.field.fieldLength / 4.0, -ai_data.field.fieldWidth / 4.0),
      rhoban_geometry::Point(-ai_data.field.fieldLength / 4.0, -ai_data.field.fieldWidth / 4.0),
      rhoban_geometry::Point(-ai_data.field.fieldLength / 4.0, ai_data.field.fieldWidth / 4.0),
  });
}

double GameInformations::fieldWidth() const
{
  return ai_data.field.fieldWidth;
}

double GameInformations::fieldHeight() const
{
  return ai_data.field.fieldLength;
}

rhoban_geometry::Point GameInformations::fieldSW() const
{
  return rhoban_geometry::Point(-ai_data.field.fieldLength / 2.0, -ai_data.field.fieldWidth / 2.0);
}
rhoban_geometry::Point GameInformations::fieldNW() const
{
  return rhoban_geometry::Point(ai_data.field.fieldLength / 2.0, -ai_data.field.fieldWidth / 2.0);
}
rhoban_geometry::Point GameInformations::fieldNE() const
{
  return rhoban_geometry::Point(ai_data.field.fieldLength / 2.0, ai_data.field.fieldWidth / 2.0);
}
rhoban_geometry::Point GameInformations::fieldSE() const
{
  return rhoban_geometry::Point(-ai_data.field.fieldLength / 2.0, ai_data.field.fieldWidth / 2.0);
}

Box GameInformations::field() const
{
  return Box(fieldSW(), fieldNE());
}

Box GameInformations::allyPenaltyArea() const
{
  return Box(
      { -ai_data.field.fieldLength / 2.0, -ai_data.field.penaltyAreaWidth / 2.0 },
      { -(ai_data.field.fieldLength / 2.0 - ai_data.field.penaltyAreaDepth), ai_data.field.penaltyAreaWidth / 2.0 });
}

Box GameInformations::opponentPenaltyArea() const
{
  return Box(
      { (ai_data.field.fieldLength / 2.0 - ai_data.field.penaltyAreaDepth), -ai_data.field.penaltyAreaWidth / 2.0 },
      { ai_data.field.fieldLength / 2.0, ai_data.field.penaltyAreaWidth / 2.0 });
}

double GameInformations::penaltyAreaWidth() const
{
  return ai_data.field.penaltyAreaWidth;
}

double GameInformations::penaltyAreaHeight() const
{
  return ai_data.field.penaltyAreaDepth;
}

bool GameInformations::infraRed(int robot_number, vision::Team team) const
{
  return getRobot(robot_number, team).infra_red;
}

};  // namespace rhoban_ssl
