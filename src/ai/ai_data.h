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

#pragma once

#include <debug.h>
#include <map>
#include <list>
#include <rhoban_utils/angle.h>
#include <rhoban_utils/timing/time_stamp.h>
#include <physic/movement_sample.h>
#include <vision/vision_data.h>
#include <physic/movement.h>
#include <math/frame_changement.h>
#include <math/position.h>

namespace rhoban_ssl
{
namespace ai
{
struct RobotPlacement
{
  bool goal_is_placed;
  std::vector<Position> field_robot_position;
  Position goalie_position;

  RobotPlacement();
  RobotPlacement(std::vector<Position> field_robot_position, Position goalie_position);
  RobotPlacement(std::vector<Position> field_robot_position);
};

typedef enum
{
  Yellow,
  Blue,
  Unknown
} Team;

class Object
{
public:
  vision::Object vision_data;
  rhoban_ssl::Movement* movement;

public:
  int id() const
  {
    return vision_data.id;
  }

  Object();
  Object(const Object& object);
  Object& operator=(const Object& object);

  void setVisionData(const vision::Object& vision_data);
  void setMovement(Movement* movement);

  // We assume that v1 and v2 are orthonormal
  void changeFrame(const rhoban_geometry::Point& origin, const Vector2d& v1, const Vector2d& v2);

  bool isPresentInVision() const;

  const rhoban_ssl::Movement& getMovement() const;
  virtual ~Object();
};

std::ostream& operator<<(std::ostream& out, const Object& object);

class Robot : public Object
{
public:
  bool is_goalie;
  bool infra_red;
  Robot();
};
class Ball : public Object
{
};

struct Field : vision::Field
{
  bool isInside(const rhoban_geometry::Point& point) const
  {
    if (not(present))
    {
      return false;
    }
    return (std::fabs(point.getX()) < (fieldLength / 2.0 + boundaryWidth) and
            std::fabs(point.getY()) < (fieldWidth / 2.0 + boundaryWidth));
  }
};

struct Constants
{
  static constexpr int NB_OF_ROBOTS_BY_TEAM = 16;

  bool enable_movement_with_integration;
  bool is_in_simulation;

  int frame_per_second;
  double period;

  double robot_radius;
  double radius_ball;
  Vector2d waiting_goal_position;
  int default_goalie_id;

  // PID for translation
  double p_translation;
  double i_translation;
  double d_translation;
  // PID for orientation
  double p_orientation;
  double i_orientation;
  double d_orientation;

  bool enable_kicking;

  double penalty_rayon;
  double translation_velocity_limit;
  double rotation_velocity_limit;
  double translation_acceleration_limit;
  double rotation_acceleration_limit;

  double time_limit_between_collision;
  double security_acceleration_ratio;
  double obstacle_avoidance_ratio;

  double radius_security_for_collision;
  double radius_security_for_avoidance;

  double wheel_radius;
  double wheel_excentricity;
  double wheel_nb_turns_acceleration_limit;

  double rules_avoidance_distance;
  double convergence_coefficient;
  double coefficient_to_increase_avoidance_convergence;

  std::vector<std::vector<double>> kick_settings;

  void load(const std::string& config_path);

  Constants(const std::string& config_path, bool is_in_simulation);
};

class AiData
{
public:
  double time_shift_with_vision;
  double time;  //(Write for Ai)
  double dt;    //(Write for Ai)

  bool force_ball_avoidance;  // This field is used by rhobot_behavior::Navigaion_inside_the_fiekd.
  std::string team_name;
  ai::Team team_color;

  AiData(const std::string& config_path, bool is_in_simulation, ai::Team team_color);

  typedef std::map<int, Robot> RobotsTable;
  typedef std::map<vision::Team, RobotsTable> RobotsTableByTeam;
  RobotsTableByTeam robots;

  std::vector<std::pair<vision::Team, Robot*> > all_robots;

  Ball ball;
  Field field;

  RobotPlacement defaultAttackingKickoffPlacement() const;
  RobotPlacement defaultDefendingKickoffPlacement() const;

  /*
   * convert a linear position [x,y] in the inetrval [-1,1]X[-1,1]
   * to an absolute position in the field.
   */
  rhoban_geometry::Point relative2absolute(double x, double y) const;
  rhoban_geometry::Point relative2absolute(const rhoban_geometry::Point& point) const;

  // the key is a pair of robot identifeds.
  typedef std::map<std::pair<int, int>, double> Collision_times_table;

  Collision_times_table table_of_collision_times;

  FrameChangement team_point_of_view;

  void changeFrameForAllObjects(const rhoban_geometry::Point& origin, const Vector2d& v1, const Vector2d& v2);
  void changeTeamColor(ai::Team team_color);

  Constants constants;

  void update(const vision::VisionData vision_data);

  // Rturn true is the robot is ready and inside the field
  bool robotIsValid(int robot_id) const;
  bool robotIsInsideTheField(int robot_id) const;

  void visitAllPairOfRobots(
      std::function<void(vision::Team robot_team_1, Robot& robot_1, vision::Team robot_team_2, Robot& robot_2)>
          visitor);

  const Collision_times_table& getTableOfCollisionTimes() const;

  void computeTableOfCollisionTimes();
  std::list<std::pair<int, double> > getCollisions(int robot_id, const Vector2d& ctrl) const;
};

}  // namespace ai
}  // namespace rhoban_ssl
