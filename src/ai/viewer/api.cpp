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
#include <debug.h>

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

void Api::addViewerPacket(char* viewer_packet)
{
  Json::Value root;
  Json::Reader reader;
  assert(reader.parse(viewer_packet, root));
  viewer_packets_.push(root);
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
  packet["field"]["circle"]["radius"] = field.circle_center_.getRadius();
  packet["field"]["circle"]["x"] = field.circle_center_.getCenter().getX();
  packet["field"]["circle"]["y"] = field.circle_center_.getCenter().getY();

  packet["informations"]["simulation"] = ai::Config::is_in_simulation;
  packet["informatons"]["color_ally"] = ai::Config::we_are_blue;

  addPacket(packet);
}

void Api::generateEntityPacket()
{
  Json::Value packet;
  double time = GlobalDataSingleThread::singleton_.ai_data_.time;

  // Ball packet
  const rhoban_geometry::Point& ball_position =
      GlobalDataSingleThread::singleton_.ball_.getMovement().linearPosition(time);
  packet["ball"]["x"] = ball_position.getX();
  packet["ball"]["y"] = ball_position.getY();

  // Robot packet
  // #dbdd56 : Yellow color && #2393c6 : Blue color
  std::string color_ally = ai::Config::we_are_blue ? "#2393c6" : "#dbdd56";
  std::string color_opponent = ai::Config::we_are_blue ? "#dbdd56" : "#2393c6";

  for (int team = 0; team < 2; team++)
  {
    std::string team_side = team == 0 ? "ally" : "opponent";
    std::string team_color = team == 0 ? color_ally : color_opponent;

    for (int rid = 0; rid < ai::Config::NB_OF_ROBOTS_BY_TEAM; rid++)
    {
      const data::Robot& current_robot = GlobalDataSingleThread::singleton_.robots_[team][rid];
      const rhoban_geometry::Point& robot_position = current_robot.getMovement().linearPosition(time);

      packet[team_side][rid]["x"] = robot_position.getX();
      packet[team_side][rid]["y"] = robot_position.getY();
      packet[team_side][rid]["orientation"] = current_robot.getMovement().angularPosition(time).value();
      packet[team_side][rid]["is_present"] = current_robot.isActive();
      packet[team_side][rid]["id"] = current_robot.id;

      if (!ai::Config::is_in_simulation)
      {
        // Activate electronics.
        packet[team_side][rid]["electronics"]["voltage"] = current_robot.electronics.voltage;
        packet[team_side][rid]["electronics"]["cap_volt"] = current_robot.electronics.cap_volt;
      }
    }
  }

  addPacket(packet);
}

void Api::addListPacket(std::shared_ptr<manager::Manager> manager)
{
  // WIP : Add in the packet.
  // const std::list<std::string>& list_of_avaible_manager = rhoban_ssl::manager::Factory::availableManagers();
  // WIP : Add List for strategy.
  // WIP : Prepare for robot behavior.
}

void Api::readViewerPacket()
{
  while (!viewer_packets_.empty())
  {
    Json::Value viewer_packet = viewer_packets_.front();
    if (viewer_packet["action"] != "")
    {
      if (viewer_packet["action"] == "emergency")
      {
        AICommander* commander = GlobalDataSingleThread::singleton_.ai_data_.commander_;
        commander->stopAll();
        commander->flush();
      }
      else
      {
        DEBUG("Aucune action trouvé");
      }
    }
    viewer_packets_.pop();
  }
}

}  // namespace viewer
}  // namespace rhoban_ssl
