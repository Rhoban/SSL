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

namespace rhoban_ssl
{
namespace viewer
{
Api Api::api_singleton_;

Api::Api()
{
}

Api& Api::getApi()
{
  return Api::api_singleton_;
}

void Api::addPacket(Json::Value& packet)
{
  packets_.push(packet);
}
std::queue<Json::Value>& Api::getQueue()
{
  return packets_;
}

void Api::generateGamePacket()
{
  Json::Value packet;
  data::Field field = GlobalDataSingleThread::singleton_.field_;

  packet["field"]["field_width"] = field.field_width_;
  packet["field"]["field_length"] = field.field_length_;

  packet["field"]["boundary_width"] = field.boundary_width_;

  packet["field"]["goal_width"] = field.goal_width_;
  packet["field"]["goal_depth"] = field.goal_depth_;

  packet["field"]["penalty_area_width"] = field.penalty_area_width_;
  packet["field"]["penalty_area_depth"] = field.penalty_area_depth_;

  packet["field"]["circle"]["radius"] = field.cirlcle_center_.getRadius();
  packet["field"]["circle"]["x"] = field.cirlcle_center_.getCenter().getX();
  packet["field"]["circle"]["y"] = field.cirlcle_center_.getCenter().getY();
  packet["field"]["simulation"] = ai::Config::is_in_simulation;

  // packet.mutable_game()->mutable_information()->set_isblue(ai::Config::we_are_blue);

  addPacket(packet);
}

void Api::generateEntityPacket()
{
  AIPacket packet;
  double time = GlobalDataSingleThread::singleton_.ai_data_.time;

  rhoban_geometry::Point ball_position = GlobalDataSingleThread::singleton_.ball_.getMovement().linearPosition(time);
  packet.mutable_entities()->mutable_ball()->set_x(ball_position.getX());
  packet.mutable_entities()->mutable_ball()->set_y(ball_position.getY());

  // Robot Location

  for (int team = 0; team < 2; team++)
  {
    for (int rid = 0; rid < ai::Config::NB_OF_ROBOTS_BY_TEAM; rid++)
    {
      Robot* robot_packet = packet.mutable_entities()->add_robot();

      const data::Robot& current_robot = GlobalDataSingleThread::singleton_.robots_[team][rid];
      const rhoban_geometry::Point& robot_position = current_robot.getMovement().linearPosition(time);
      double robot_direction = current_robot.getMovement().angularPosition(time).value();

      robot_packet->set_x(robot_position.getX());
      robot_packet->set_y(robot_position.getY());
      robot_packet->set_isally(!team);
      robot_packet->set_ispresent(current_robot.isActive());
      robot_packet->set_robot_id(current_robot.id);
      robot_packet->set_dir(robot_direction);
    }
  }

  // addPacket(packet);
}

void Api::addListPacket(std::shared_ptr<manager::Manager> manager)
{
  // WIP : Add in the packet.
  // const std::list<std::string>& list_of_avaible_manager = rhoban_ssl::manager::Factory::availableManagers();
  // WIP : Add List for strategy.
  // WIP : Prepare for robot behavior.
}

void addElectronicsPacket()
{
}

}  // namespace viewer
}  // namespace rhoban_ssl
