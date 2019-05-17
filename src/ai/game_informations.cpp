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
GameInformations::GameInformations()
{
}

GameInformations::~GameInformations()
{
}

double GameInformations::time() const
{
  return GlobalDataSingleThread::singleton_.ai_data_.time;
}

rhoban_geometry::Point GameInformations::allyGoalCenter() const
{
  return rhoban_geometry::Point(-GlobalDataSingleThread::singleton_.field_.field_length_ / 2.0, 0.0);
}

rhoban_geometry::Point GameInformations::opponentGoalCenter() const
{
  return rhoban_geometry::Point(GlobalDataSingleThread::singleton_.field_.field_length_ / 2.0, 0.0);
}

rhoban_geometry::Point GameInformations::centerMark() const
{
  return rhoban_geometry::Point(0.0, 0.0);
}

rhoban_geometry::Point GameInformations::opponentCornerRight() const
{
  return rhoban_geometry::Point(GlobalDataSingleThread::singleton_.field_.field_length_ / 2.0,
                                -GlobalDataSingleThread::singleton_.field_.field_width_ / 2.0);
}

rhoban_geometry::Point GameInformations::opponentCornerLeft() const
{
  return rhoban_geometry::Point(GlobalDataSingleThread::singleton_.field_.field_length_ / 2.0,
                                GlobalDataSingleThread::singleton_.field_.field_width_ / 2.0);
}

const data::Robot& GameInformations::getRobot(int robot_number, Team team) const
{
  return GlobalDataSingleThread::singleton_.robots_[team][robot_number];
}

void GameInformations::getRobotInLine(const rhoban_geometry::Point p1, const rhoban_geometry::Point p2, Team team,
                                      double distance, std::vector<int>& result) const
{
  if (normSquare(p1 - p2) == 0)
  {
    return;
  }

  for (size_t i = 0; i < ai::Config::NB_OF_ROBOTS_BY_TEAM; i++)
  {
    const data::Robot& robot = getRobot(i, team);
    if (robot.isActive())
    {
      const rhoban_geometry::Point& robot_position = robot.getMovement().linearPosition(time());
      if (distanceFromPointToLine(robot_position, p1, p2) <= distance)
      {
        result.push_back(i);
      }
    }
  }
}

std::vector<int> GameInformations::getRobotInLine(const rhoban_geometry::Point p1, const rhoban_geometry::Point p2,
                                                  Team team, double distance) const
{
  std::vector<int> result;
  getRobotInLine(p1, p2, team, distance, result);
  return result;
}

std::vector<int> GameInformations::getRobotInLine(const rhoban_geometry::Point p1, const rhoban_geometry::Point p2,
                                                  double distance) const
{
  std::vector<int> result;
  getRobotInLine(p1, p2, Ally, distance, result);
  getRobotInLine(p1, p2, Opponent, distance, result);
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
      rhoban_geometry::Point(GlobalDataSingleThread::singleton_.field_.field_length_ / 2.0,
                             GlobalDataSingleThread::singleton_.field_.goal_width_ / 2.0);
  const rhoban_geometry::Point right_post_position =
      rhoban_geometry::Point(GlobalDataSingleThread::singleton_.field_.field_length_ / 2.0,
                             -GlobalDataSingleThread::singleton_.field_.goal_width_ / 2.0);
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
    std::vector<int> robot_in_line = GameInformations::getRobotInLine(point, analysed_point, Opponent, 0.15);
    std::vector<int> robot_in_line2 = GameInformations::getRobotInLine(point, analysed_point, Ally, 0.15);
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

int GameInformations::getShirtNumberOfClosestRobotToTheBall(Team team) const
{
  return getShirtNumberOfClosestRobot(team, ballPosition());
}

int GameInformations::getShirtNumberOfClosestRobot(Team team, rhoban_geometry::Point point) const
{
  int id = -1;
  double distance_max = -1;
  for (int i = 0; i < ai::Config::NB_OF_ROBOTS_BY_TEAM; i++)
  {
    const data::Robot& robot = getRobot(i, team);
    if (robot.isActive())
    {
      const rhoban_geometry::Point& robot_position = robot.getMovement().linearPosition(time());
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

double GameInformations::getRobotDistanceFromAllyGoalCenter(int robot_number, Team team) const
{
  double distance = -1;
  const data::Robot& robot = getRobot(robot_number, team);
  if (robot.isActive())
  {
    const rhoban_geometry::Point& robot_position = robot.getMovement().linearPosition(time());
    Vector2d goal_center_robot = robot_position - allyGoalCenter();
    distance = goal_center_robot.norm();
    distance = (GlobalDataSingleThread::singleton_.field_.field_length_ - distance) /
               GlobalDataSingleThread::singleton_.field_.field_length_;
  }
  return distance;
}

std::vector<double> GameInformations::threat(Team team) const
{
  std::vector<double> v_threat;
  for (size_t i = 0; i < ai::Config::NB_OF_ROBOTS_BY_TEAM; i++)
  {
    double threat = getRobotDistanceFromAllyGoalCenter(i, team);
    v_threat.push_back(threat);
  }
  return v_threat;
}

int GameInformations::shirtNumberOfThreatMax(Team team) const
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

int GameInformations::shirtNumberOfThreatMax2(Team team) const
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

const data::Ball& GameInformations::ball() const
{
  return GlobalDataSingleThread::singleton_.ball_;
}

rhoban_geometry::Point GameInformations::ballPosition() const
{
  return ball().getMovement().linearPosition(time());
}

rhoban_geometry::Point GameInformations::centerAllyField() const
{
  return rhoban_geometry::Point(-GlobalDataSingleThread::singleton_.field_.field_length_ / 4.0, 0.0);
}
rhoban_geometry::Point GameInformations::centerOpponentField() const
{
  return rhoban_geometry::Point(GlobalDataSingleThread::singleton_.field_.field_length_ / 4.0, 0.0);
}

double GameInformations::getRobotRadius() const
{
  return ai::Config::robot_radius;
}

double GameInformations::getBallRadius() const
{
  return ai::Config::ball_radius;
}

std::vector<rhoban_geometry::Point> GameInformations::centerQuarterField() const
{
  return std::vector<rhoban_geometry::Point>({
      rhoban_geometry::Point(GlobalDataSingleThread::singleton_.field_.field_length_ / 4.0,
                             GlobalDataSingleThread::singleton_.field_.field_width_ / 4.0),
      rhoban_geometry::Point(GlobalDataSingleThread::singleton_.field_.field_length_ / 4.0,
                             -GlobalDataSingleThread::singleton_.field_.field_width_ / 4.0),
      rhoban_geometry::Point(-GlobalDataSingleThread::singleton_.field_.field_length_ / 4.0,
                             -GlobalDataSingleThread::singleton_.field_.field_width_ / 4.0),
      rhoban_geometry::Point(-GlobalDataSingleThread::singleton_.field_.field_length_ / 4.0,
                             GlobalDataSingleThread::singleton_.field_.field_width_ / 4.0),
  });
}

double GameInformations::fieldWidth() const
{
  return GlobalDataSingleThread::singleton_.field_.field_width_;
}

double GameInformations::fieldHeight() const
{
  return GlobalDataSingleThread::singleton_.field_.field_length_;
}

rhoban_geometry::Point GameInformations::fieldSW() const
{
  return rhoban_geometry::Point(-GlobalDataSingleThread::singleton_.field_.field_length_ / 2.0,
                                -GlobalDataSingleThread::singleton_.field_.field_width_ / 2.0);
}
rhoban_geometry::Point GameInformations::fieldNW() const
{
  return rhoban_geometry::Point(GlobalDataSingleThread::singleton_.field_.field_length_ / 2.0,
                                -GlobalDataSingleThread::singleton_.field_.field_width_ / 2.0);
}
rhoban_geometry::Point GameInformations::fieldNE() const
{
  return rhoban_geometry::Point(GlobalDataSingleThread::singleton_.field_.field_length_ / 2.0,
                                GlobalDataSingleThread::singleton_.field_.field_width_ / 2.0);
}
rhoban_geometry::Point GameInformations::fieldSE() const
{
  return rhoban_geometry::Point(-GlobalDataSingleThread::singleton_.field_.field_length_ / 2.0,
                                GlobalDataSingleThread::singleton_.field_.field_width_ / 2.0);
}

Box GameInformations::field() const
{
  return Box(fieldSW(), fieldNE());
}

Box GameInformations::allyPenaltyArea() const
{
  return Box({ -GlobalDataSingleThread::singleton_.field_.field_length_ / 2.0,
               -GlobalDataSingleThread::singleton_.field_.penalty_area_width_ / 2.0 },
             { -(GlobalDataSingleThread::singleton_.field_.field_length_ / 2.0 -
                 GlobalDataSingleThread::singleton_.field_.penalty_area_depth_),
               GlobalDataSingleThread::singleton_.field_.penalty_area_width_ / 2.0 });
}

Box GameInformations::opponentPenaltyArea() const
{
  return Box({ (GlobalDataSingleThread::singleton_.field_.field_length_ / 2.0 -
                GlobalDataSingleThread::singleton_.field_.penalty_area_depth_),
               -GlobalDataSingleThread::singleton_.field_.penalty_area_width_ / 2.0 },
             { GlobalDataSingleThread::singleton_.field_.field_length_ / 2.0,
               GlobalDataSingleThread::singleton_.field_.penalty_area_width_ / 2.0 });
}

double GameInformations::penaltyAreaWidth() const
{
  return GlobalDataSingleThread::singleton_.field_.penalty_area_width_;
}

double GameInformations::penaltyAreaHeight() const
{
  return GlobalDataSingleThread::singleton_.field_.penalty_area_depth_;
}

bool GameInformations::infraRed(int robot_number, Team team) const
{
  return getRobot(robot_number, team).infraRed();
}

};  // namespace rhoban_ssl
