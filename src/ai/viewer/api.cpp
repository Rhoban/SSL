/*
    This file is part of SSL.

    Copyright 2019 Schmitz Etienne (hello@etienne-schmitz.com)

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

#include "api.h"
#include <rhoban_utils/timing/time_stamp.h>

namespace rhoban_ssl
{
namespace viewer
{
Api Api::api_singleton_;

Api::Api() : Task()
{
}

Api Api::getApi()
{
  return Api::api_singleton_;
}

void Api::addPacket(AIPacket packet)
{
  packets_.push(packet);
}
std::queue<AIPacket>* Api::getQueue()
{
  return &packets_;
}

void Api::updateField()
{
  AIPacket packet;
  FieldPacket field_packet;
  rhoban_ssl::vision::Field field = GlobalDataSingleThread::singleton_.vision_data_.field_;

  // Prepare packet
  field_packet.set_fieldlength(field.fieldLength);
  field_packet.set_fieldwidth(field.fieldWidth);
  field_packet.set_boundarywidth(field.boundaryWidth);
  field_packet.set_goaldepth(field.goalDepth);
  field_packet.set_goalwidth(field.goalWidth);
  field_packet.set_penaltyareadepth(field.penaltyAreaDepth);
  field_packet.set_penaltyareawidth(field.penaltyAreaWidth);
  packet.set_allocated_field(&field_packet);

  // Send packet
  packets_.push(packet);
}

bool Api::runTask(void)
{
  AIPacket packet;
  LocationPacket location_packet;
  BallLocation location_ball;

  time_ = rhoban_utils::TimeStamp::now().getTimeMS() / 1000.0;
  rhoban_ssl::vision::Ball ball = GlobalDataSingleThread::singleton_.vision_data_.ball_;

  // Ball Location
  location_ball.set_x(ball.movement.linearPosition(time_).getX());
  location_ball.set_y(ball.movement.linearPosition(time_).getY());
  location_packet.set_allocated_ball(&location_ball);

  // Robot Location
  rhoban_ssl::vision::Robot robots[2][ai::Config::NB_OF_ROBOTS_BY_TEAM] =
      GlobalDataSingleThread::singleton_.vision_data_.robots_;

  for (int team = 0; team < 2; team++)
  {
    for (int id = 0; id < ai::Config::NB_OF_ROBOTS_BY_TEAM; id++)
    {
      rhoban_ssl::vision::Robot robot = robots[team][id];
      RobotLocation location_robot = *location_packet.add_robot();
      location_robot.set_robot_id(robot.id);
      location_robot.set_x(robot.movement.linearPosition(time_).getX());
      location_robot.set_y(robot.movement.linearPosition(time_).getY());
      // 0 -> Ally || 1 -> Opponent
      location_robot.set_isally(!team);
      location_robot.set_dir(robot.movement.angularPosition(time_).value());
    }
  }

  packets_.push(packet);

  return true;
}

}  // namespace viewer
}  // namespace rhoban_ssl
