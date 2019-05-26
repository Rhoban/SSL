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
#include "viewer_communication.h"
#include <viewer_client.h>
#include <manager/factory.h>
#include <core/collection.h>

namespace rhoban_ssl
{
namespace viewer
{
ViewerCommunication::ViewerCommunication(AI* ai)
  : ai_(ai), last_sending_time_(rhoban_utils::TimeStamp::now().getTimeSec())
{
}

bool ViewerCommunication::runTask()
{
  // processIncomingPackets();

  if (GlobalDataSingleThread::singleton_.ai_data_.time - last_sending_time_ > SENDING_DELAY)
  {
    sendViewerPackets();
    last_sending_time_ = GlobalDataSingleThread::singleton_.ai_data_.time;
  }

  return true;
}

void ViewerCommunication::processIncomingPackets()
{
  while (!viewer::ViewerDataGlobal::get().received_packets.empty())
  {
    Json::Value viewer_packet = viewer::ViewerDataGlobal::get().received_packets.front();
    if (viewer_packet["action"] != "")
    {
      if (viewer_packet["action"] == "emergency")
      {
        ai_->emergency();
      }
      else
      {
        DEBUG("Aucune action trouvé");
      }
    }
    viewer::ViewerDataGlobal::get().received_packets.pop();
  }
}

void ViewerCommunication::sendViewerPackets()
{
  // GlobalData status
  viewer::ViewerDataGlobal::get().packets_to_send.push(fieldPacket());
  viewer::ViewerDataGlobal::get().packets_to_send.push(ballPacket());
  viewer::ViewerDataGlobal::get().packets_to_send.push(teamsPacket());
  viewer::ViewerDataGlobal::get().packets_to_send.push(refereePacket());
  viewer::ViewerDataGlobal::get().packets_to_send.push(informationsPacket());
  viewer::ViewerDataGlobal::get().packets_to_send.push(aiPacket());
}

Json::Value ViewerCommunication::fieldPacket()
{
  Json::Value packet;
  const data::Field& field = GlobalDataSingleThread::singleton_.field_;

  packet["field"]["length"] = field.field_length_;
  packet["field"]["width"] = field.field_width_;
  packet["field"]["boundary_width"] = field.boundary_width_;
  packet["field"]["goal"]["width"] = field.goal_width_;
  packet["field"]["goal"]["depth"] = field.goal_depth_;
  packet["field"]["penalty_area"]["width"] = field.penalty_area_width_;
  packet["field"]["penalty_area"]["depth"] = field.penalty_area_depth_;
  packet["field"]["circle"]["x"] = field.circle_center_.getCenter().getX();
  packet["field"]["circle"]["y"] = field.circle_center_.getCenter().getY();
  packet["field"]["circle"]["radius"] = field.circle_center_.getRadius();

  return packet;
}

Json::Value ViewerCommunication::ballPacket()
{
  Json::Value packet;

  const rhoban_geometry::Point& ball_position = GlobalDataSingleThread::singleton_.ball_.getMovement().linearPosition(
      GlobalDataSingleThread::singleton_.ai_data_.time);
  packet["ball"]["position"]["x"] = ball_position.getX();
  packet["ball"]["position"]["y"] = ball_position.getY();

  const Vector2d& ball_velocity = GlobalDataSingleThread::singleton_.ball_.getMovement().linearVelocity(
      GlobalDataSingleThread::singleton_.ai_data_.time);
  packet["ball"]["velocity"]["x"] = ball_velocity.getX();
  packet["ball"]["velocity"]["y"] = ball_velocity.getY();

  packet["ball"]["radius"] = ai::Config::ball_radius;

  return packet;
}

Json::Value ViewerCommunication::teamsPacket()
{
  Json::Value packet;
  double time = GlobalDataSingleThread::singleton_.ai_data_.time;
  const data::Referee& referee = GlobalDataSingleThread::singleton_.referee_;

  const std::string blue_color = "#2393c6";
  const std::string yellow_color = "#dbdd56";

  for (uint team_id = 0; team_id < 2; team_id++)
  {
    bool ally_info = (team_id == Ally);
    std::string team = ally_info ? "allies" : "opponents";

    // referee informations
    if (ally_info)
    {
      packet["teams"][team]["positive_axis"] = referee.allyOnPositiveHalf();
    }
    else
    {
      packet["teams"][team]["positive_axis"] = !GlobalDataSingleThread::singleton_.referee_.allyOnPositiveHalf();
    }
    packet["teams"][team]["name"] = referee.teams_info[team_id].name;
    packet["teams"][team]["score"] = referee.teams_info[team_id].score;
    packet["teams"][team]["timeout"]["remaincount"] = referee.teams_info[team_id].timeout_remaining_count;
    packet["teams"][team]["timeout"]["remaining_time"] = referee.teams_info[team_id].timeout_remaining_time;
    packet["teams"][team]["goalkeeper_number"] = referee.teams_info[team_id].goalkeeper_number;

    packet["teams"][team]["cards"]["yellow"] = referee.teams_info[team_id].yellow_cards_count;
    for (uint i = 0; i < referee.teams_info[team_id].yellow_card_times.size(); ++i)
    {
      packet["teams"][team]["cards"]["yellow"]["time"][i] = referee.teams_info[team_id].yellow_card_times.at(i);
    }
    packet["teams"][team]["cards"]["red"] = referee.teams_info[team_id].red_cards_count;
    packet["teams"][team]["fouls"] = referee.teams_info[team_id].foul_counter;
    packet["teams"][team]["max_allowed_bots"] = referee.teams_info[team_id].max_allowed_bots;

    // robots informations
    for (uint rid = 0; rid < ai::Config::NB_OF_ROBOTS_BY_TEAM; rid++)
    {
      const data::Robot& current_robot = GlobalDataSingleThread::singleton_.robots_[team_id][rid];
      const rhoban_geometry::Point& robot_position = current_robot.getMovement().linearPosition(time);

      packet["teams"][team]["bots"][rid]["number"] = current_robot.id;
      packet["teams"][team]["bots"][rid]["time"] = current_robot.getMovement().lastTime();

      packet["teams"][team]["bots"][rid]["position"]["x"] = robot_position.getX();
      packet["teams"][team]["bots"][rid]["position"]["y"] = robot_position.getY();
      packet["teams"][team]["bots"][rid]["position"]["orientation"] =
          current_robot.getMovement().angularPosition(GlobalDataSingleThread::singleton_.ai_data_.time).value();

      packet["teams"][team]["bots"][rid]["velocity"]["x"] =
          current_robot.getMovement().linearVelocity(GlobalDataSingleThread::singleton_.ai_data_.time).getX();
      packet["teams"][team]["bots"][rid]["velocity"]["y"] =
          current_robot.getMovement().linearVelocity(GlobalDataSingleThread::singleton_.ai_data_.time).getY();
      packet["teams"][team]["bots"][rid]["velocity"]["theta"] =
          current_robot.getMovement().angularVelocity(GlobalDataSingleThread::singleton_.ai_data_.time).value();

      // todo
      packet["teams"][team]["bots"][rid]["last_control"]["time"] = 0;
      packet["teams"][team]["bots"][rid]["last_control"]["velocity"]["x"] = 0;
      packet["teams"][team]["bots"][rid]["last_control"]["velocity"]["y"] = 0;
      packet["teams"][team]["bots"][rid]["last_control"]["velocity"]["theta"] = 0;

      packet["teams"][team]["bots"][rid]["radius"] = ai::Config::robot_radius;
      packet["teams"][team]["bots"][rid]["is_present"] = current_robot.isActive();

      if (ally_info)
      {
        packet["teams"][team]["bots"][rid]["color"] = (ai::Config::we_are_blue) ? blue_color : yellow_color;
        packet["teams"][team]["bots"][rid]["behavior"] = ai_->getRobotBehaviorOf(rid);
        // todo
        // packet["teams"][team]["bots"][rid]["strategy"] = "do_nothing";

        if (!ai::Config::is_in_simulation)
        {
          // Activate electronics.
          packet["teams"][team]["bots"][rid]["electronics"]["alive"] = current_robot.robotOk();
          packet["teams"][team]["bots"][rid]["electronics"]["voltage"] = current_robot.electronics.voltage;
          packet["teams"][team]["bots"][rid]["electronics"]["cap_volt"] = current_robot.electronics.cap_volt;
          packet["teams"][team]["bots"][rid]["electronics"]["ir_trigered"] = current_robot.infraRed();
          packet["teams"][team]["bots"][rid]["electronics"]["odometry"]["x"] = current_robot.electronics.xpos;
          packet["teams"][team]["bots"][rid]["electronics"]["odometry"]["y"] = current_robot.electronics.ypos;
          packet["teams"][team]["bots"][rid]["electronics"]["odometry"]["orientation"] = current_robot.electronics.ang;
          packet["teams"][team]["bots"][rid]["electronics"]["errors"]["driver"] = current_robot.driverError();
        }
      }
      else
      {
        packet["teams"][team]["bots"][rid]["color"] = (!ai::Config::we_are_blue) ? blue_color : yellow_color;
      }
    }
  }
  return packet;
}

Json::Value ViewerCommunication::refereePacket()
{
  Json::Value packet;

  packet["referee"]["stage"]["value"] = GlobalDataSingleThread::singleton_.referee_.getCurrentStageName();
  packet["referee"]["stage"]["remaining_time"] = GlobalDataSingleThread::singleton_.referee_.stage_time_left;
  packet["referee"]["state"] = GlobalDataSingleThread::singleton_.referee_.getCurrentStateName();

  return packet;
}

Json::Value ViewerCommunication::informationsPacket()
{
  Json::Value packet;

  packet["informations"]["simulation"] = ai::Config::is_in_simulation;
  packet["informations"]["packets_per_second"] =
      1. / GlobalDataSingleThread::singleton_.ai_data_.time - last_sending_time_;

  // todo
  // packet["informatons"]["ping"] = ai::Config::we_are_blue;

  return packet;
}

Json::Value ViewerCommunication::aiPacket()
{
  Json::Value packet;

  const std::vector<std::string>& available_managers = list2vector(manager::Factory::availableManagers());
  for (uint i = 0; i < available_managers.size(); i++)
  {
    packet["ai"]["managers"]["availables"][i]["name"] = available_managers.at(i);
  }
  // todo
  // packet["ai"]["managers"]["current"]["name"] = ai_->getCurrentManager().get()

  // for ...
  // packet["ai"]["managers"]["current"]["strategies_used"][i]["name"] =

  // for ... manual
  // packet["ai"]["strategies"][id]["name"]
  // packet["ai"]["strategies"][id]["bots_required"]
  return packet;
}

}  // namespace viewer
}  // namespace rhoban_ssl
