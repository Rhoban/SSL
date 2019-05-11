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
#include <iostream>

namespace rhoban_ssl
{
namespace viewer
{
Api Api::api_singleton_;

Api::Api()
{
}

Api Api::getApi()
{
  //DEBUG(api_singleton_.packets_.size());
  return Api::api_singleton_;
}

void Api::addPacket(AIPacket packet)
{
  //DEBUG("ADD");
  //DEBUG("2222222222222");
  //std::cout << &packets_<< std::endl;
  packets_.push(packet);
}
std::queue<AIPacket>* Api::getQueue()
{
    //DEBUG("1111111111");
    //DEBUG(packets_.size());
    DEBUG("GetQueue &packet:" << &packets_);
    return &packets_;
}

void Api::updateField()
{
  AIPacket packet;
  FieldPacket field_packet;
  rhoban_ssl::vision::Field field = GlobalDataSingleThread::singleton_.vision_data_.field_;

  // Prepare packet
  /*
  field_packet.set_fieldlength(field.fieldLength);
  field_packet.set_fieldwidth(field.fieldWidth);
  field_packet.set_boundarywidth(field.boundaryWidth);
  field_packet.set_goaldepth(field.goalDepth);
  field_packet.set_goalwidth(field.goalWidth);
  field_packet.set_penaltyareadepth(field.penaltyAreaDepth);
  field_packet.set_penaltyareawidth(field.penaltyAreaWidth);
  */
  packet.mutable_field()->set_fieldlength(field.fieldLength);
  packet.mutable_field()->set_fieldwidth(field.fieldWidth);
  packet.mutable_field()->set_boundarywidth(field.boundaryWidth);
  packet.mutable_field()->set_goaldepth(field.goalDepth);
  packet.mutable_field()->set_goalwidth(field.goalWidth);
  packet.mutable_field()->set_penaltyareadepth(field.penaltyAreaDepth);
  packet.mutable_field()->set_penaltyareawidth(field.penaltyAreaWidth);


  //packet.set_allocated_field(&field_packet);
  //packet.set_allocated_field(&field_packet);
  addPacket(packet);
  packet.release_field();
  DEBUG("updateField END &packet:" << &packets_);
}

void Api::updateLocationPacket(double time)
{

  AIPacket packet;
  LocationPacket location_packet;
  BallLocation location_ball;

  //DEBUG(time);
  //time_ = time;
  //DEBUG(time_);
  //DEBUG("------------------------------");

  //rhoban_ssl::vision::Ball ball = GlobalDataSingleThread::singleton_.vision_data_.ball_;
  //DEBUG(ball);
  /**
  location_ball.set_x(1);
  location_ball.set_y(1);
  location_packet.set_allocated_ball(&location_ball);
  */

  // Ball Location
  //location_ball.set_x(1);
  //location_ball.set_y(1);
  //location_packet.set_allocated_ball(&location_ball);


  // Robot Location
  //rhoban_ssl::vision::Robot robots[2][ai::Config::NB_OF_ROBOTS_BY_TEAM] =
  //    GlobalDataSingleThread::singleton_.vision_data_.robots_;

  /**
  for (int team = 0; team < 2; team++)
  {
    for (int id = 0; id < ai::Config::NB_OF_ROBOTS_BY_TEAM; id++)
    {
      rhoban_ssl::vision::Robot robot = robots[team][id];
      RobotLocation location_robot = *location_packet.add_robot();
      location_robot.set_robot_id(robot.id);
      location_robot.set_x(robot.movement.linearPosition().getX());
      location_robot.set_y(robot.movement.linearPosition().getY());
      // 0 -> Ally || 1 -> Opponent
      location_robot.set_isally(!team);
      location_robot.set_dir(robot.movement.angularPosition().value());
    }
  }*/

  //DEBUG(packets_.size());
  //DEBUG("add Packet");
  //packets_.push(packet);
  //DEBUG(packets_.size());
}

}  // namespace viewer
}  // namespace rhoban_ssl
