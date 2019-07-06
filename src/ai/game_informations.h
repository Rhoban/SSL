/*
    This file is part of SSL.

    Copyright 2018 Boussicault Adrien (adrien.boussicault@u-bordeaux.fr)
    Copyright 2018 Bezamat Jérémy (jeremy.bezamat@gmail.com)
    Copyright 2018 Muller Xavier (xavier.muller@etu.u-bordeaux.fr)

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
#pragma once
#include <math/box.h>
#include <data.h>

namespace rhoban_ssl
{
class GameInformations
{
public:
  GameInformations();
  virtual ~GameInformations();

  double time() const;

  /**************************  Ball INFORMATIONS ***************************/
  /**
   * @brief returns the ball.
   * @return a ball.
   */
  const data::Ball& ball() const;
  /**
   * @brief returns the current ball's position.
   * @return const rhoban_geometry::Point
   */
  rhoban_geometry::Point ballPosition() const;
  /**
   * @brief returns the ball's radius.
   * @return a real.
   */
  double getBallRadius() const;

  /**************************  Field INFORMATIONS ***************************/
  /**
   * @brief returns a box that contains the coordinates of the corners of the
   *        field.
   * @return a box
   */
  Box field() const;
  /**
   * @brief returns the position of the center mark of the field.
   * @return a point
   */
  rhoban_geometry::Point centerMark() const;
  /**
   * @brief returns a box that represent the ally penalty area.
   * @return a box
   */
  Box allyPenaltyArea() const;
  /**
   * @brief returns a box that represent the opponent penalty area.
   * @return a box
   */
  Box opponentPenaltyArea() const;

  /**
   * @brief returns an array which contains the center position
   * of the four field's quarter.
   * @return a point
   */
  std::vector<rhoban_geometry::Point> centerQuarterField() const;
  /**
   * @brief returns the position of the center ally field.
   * @return a point
   */
  rhoban_geometry::Point centerAllyField() const;
  /**
   * @brief returns the position of the center opponant field.
   * @return a point
   */
  rhoban_geometry::Point centerOpponentField() const;
  /**
   * @brief returns the field's width.
   * the width correspond to the x axis
   * @return width in meter
   */
  double fieldWidth() const;
  /**
   * @brief returns field's height.
   * the length correspond to the y axis
   * @return length in meter
   */
  double fieldHeight() const;
  /**
   * @brief returns penalty area's width.
   * the width correspond to the x axis
   * @return width in meter
   */
  double penaltyAreaWidth() const;
  /**
   * @brief returns penalty area's height.
   * the length correspond to the y axis
   * @return length in meter
   */
  double penaltyAreaHeight() const;
  /**
   * @brief returns the South West point of the field.
   * @return a point
   */
  rhoban_geometry::Point fieldSW() const;
  /**
   * @brief returns the North West point of the field.
   * @return a point
   */
  rhoban_geometry::Point fieldNW() const;
  /**
   * @brief returns the South East point of the field.
   * @return a point
   */
  rhoban_geometry::Point fieldNE() const;
  /**
   * @brief returns the South East point of the field.
   * @return a point
   */
  rhoban_geometry::Point fieldSE() const;

  /*************************  Robot INFORMATIONS ***************************/
  /**
   * @brief returns the robot's whose robot's number is given in parameter.
   *
   * Robot have two type of identification number: its number and its ID.
   * The robot's number is the same concept as a foot player's shirt number.
   * Usually during a game, there are two robots with the same number, one in the ally team
   * and another one in the opponent team.
   * All the robots have differents ids. (there is a bijection between ids).
   *
   * @param robot_number
   * the number of the robot.
   * @param team
   * the team of the robot
   * @return a reference on a robot
   */
  const data::Robot& getRobot(int robot_number, Team team = Ally) const;
  /**
   * @brief returns the robot's radius.
   * @return a radius in meter
   */
  double getRobotRadius() const;
  /**
   * @brief returns true if the ball is closed to the kicker.
   *
   * In front of the kicker there is an infrared sensor,
   * when the ball is on it, the sensor returns true.
   * In fact we don't use it to kick. Indeed we can ask the robot
   * to kick when infrared is not on, in that case the robot will
   * kick as soon as the infrared will be on.
   *
   * @param robot_number
   * the robot number
   * @see GameInformation::get_robot() to know the difference between robot'id and robot's number).
   * @param team the team of the robot (ally or opponent)
   * @return a boolean
   */
  bool infraRed(int robot_number, Team team = Ally) const;
  /**
   * @brief Computes all the robots of a given team at a distance `distance` of the line
   * ( `P1`, `P2` ).
   *
   * Those robot shirt numbers are stored in `result`.
   *
   * If `P1` == `P2` this function returns an empty list.
   *
   * @param p1
   * a point
   * @param p2
   * a point
   * @param team
   * ( Opponent or Ally)
   * @param distance (usually the robot radius)
   * @param[out] result
   * a vector of robot's number
   * @see GameInformation::get_robot() to know the difference between robot'id and robot's number).
   */
  void getRobotInLine(const rhoban_geometry::Point p1, const rhoban_geometry::Point p2, Team team, double distance,
                      std::vector<int>& result) const;
  /**
   * @brief same as void get_robot_in_line() but returns the result
   * instead of storing it in a variable.
   *
   * If `P1` == `P2` this function returns an empty list.
   * @param p1
   * a point
   * @param p2
   * a point
   * @param team
   * ( Opponent or Ally)
   * @param distance (usually the robot radius)
   * @return a vector of robot's number
   * @see GameInformation::get_robot() to know the difference between robot'id and robot's number).
   */
  std::vector<int> getRobotInLine(const rhoban_geometry::Point p1, const rhoban_geometry::Point p2,
                                  Team team = Opponent, double distance = 0.4) const;
  /**
   * @brief same as vector<int> get_robot_in_line() but doesn't
   * consider the robot's team.
   * @param p1
   * @param p2
   * @param distance (usually the robot radius)
   * @return a vector of robot's shirt number
   * @see GameInformation::get_robot() to know the difference between robot'id and robot's number).
   */
  std::vector<int> getRobotInLine(const rhoban_geometry::Point p1, const rhoban_geometry::Point p2,
                                  double distance) const;
  /**
   * @brief returns the robot's number which is closest to the point
   * given in parameter in the team given in parameter.
   * @param team
   * ( Opponent or Ally)
   * @return robot's shirt number (-1 if not found)
   * @see GameInformation::get_robot() to know the difference between robot'id and robot's number).
   */
  int getShirtNumberOfClosestRobot(Team team, rhoban_geometry::Point point) const;
  /**
   * @brief returns the robot's shirt number which is closest robot to the ball
   * from the team given in parameter.
   * @param team
   * ( Opponent or Ally)
   * @return robot's shirt number(-1 if not found)
   * @see GameInformation::get_robot() to know the difference between robot'id and robot's number).
   */
  int getShirtNumberOfClosestRobotToTheBall(Team team) const;

  /**************************  Algos INFORMATIONS *************************/
  /**
   * @brief returns the "ideal" position to target that maximazes the chance to score
   * in the targeted goal given in parameter and its probability to score.
   *
   * The algorithm tests different lines between the point and the goal given
   * in parameter and returns the leasts covered by robots line.
   * @note Attack algorithm
   * @param a point
   * (usually the ball's position)
   * @param a goal
   * (usually the ennemy's goal center)
   * @return pair<rhoban_geometry::Point, double>
   * (double correspond of his "efficiency rate" )
   */
  std::pair<rhoban_geometry::Point, double> findGoalBestMove(
      const rhoban_geometry::Point point, const rhoban_geometry::Point goal = rhoban_geometry::Point(66, 66)) const;
  /**
   * @brief returns the distance between a robot with the shirt number given in parameter
   * and the ally goal center.
   * @note Defense algorithm
   * @param robot_number
   * @see GameInformation::get_robot() to know the difference between robot'id and robot's number).
   * @param team
   * the team of the robot (opponent by default)
   * @return a distance
   */
  double getRobotDistanceFromAllyGoalCenter(int robot_number, Team team = Opponent) const;
  /**
   * @brief returns the distance between all robots in the team given in parameter and
   * the ally goal center.
   * @note Defense algorithm
   * @param team
   * (opponent by default)
   * @return a vector
   * [0] = distance of the first robot,
   * [1] = distance of the second robot
   * ...
   */
  std::vector<double> threat(Team team = Opponent) const;
  /**
   * @brief returns the robot's number of the biggest threat belonging
   * to the team given in parameter.
   *
   * It corresponds to the closest opposant's robot to the ally's goal center.
   * @note Defense algorithm
   * @param team
   * @return a robot's shirt number
   * @see GameInformation::get_robot() to know the difference between robot'id and robot's number).
   */
  int shirtNumberOfThreatMax(Team team) const;
  /**
   * @brief returns the robot's number of the second biggest threat belonging
   * to the team given in parameter.
   *
   * It corresponds to the second closest opposant's robot to the ally's goal
   * center.
   * @note Defense algorithm
   * @param team
   * @return a robot's shirt number
   * @see GameInformation::get_robot() to know the difference between robot'id and robot's number).
   */
  int shirtNumberOfThreatMax2(Team team) const;  // second threat max
};

}  // namespace rhoban_ssl
