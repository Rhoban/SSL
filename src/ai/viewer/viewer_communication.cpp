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
    sendStatusPackets();
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

void ViewerCommunication::sendStatusPackets()
{
  // GlobalData status
  viewer::ViewerDataGlobal::get().addPacket(fieldPacket());
  viewer::ViewerDataGlobal::get().addPacket(ballPacket());
  // viewer::ViewerDataGlobal::get().addPacket(teamsPacket());
  // viewer::ViewerDataGlobal::get().addPacket(mobilesStatus());

  // Information of the ai
  //  viewer::ViewerDataGlobal::get().addPacket(availableManager());
  //  viewer::ViewerDataGlobal::get().addPacket(availableRobotBehavior());
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
}

Json::Value ViewerCommunication::teamsPacket()
{
  Json::Value packet;

  const data::Referee& referee = GlobalDataSingleThread::singleton_.referee_;
  const std::string blue_color = "#2393c6";
  const std::string yellow_color = "#dbdd56";

  //  for (uint team_id = 0; team_id < 2; ++team_id)
  //  {
  //    bool ally_info = (team_id == Ally);
  //    std::string team = ally_info ? "allies" : "opponents";

  //    // referee informations
  //    if (ally_info)
  //    {
  //      packet[team]["positive_axis"] = referee.allyOnPositiveHalf();
  //    }
  //    else
  //    {
  //      packet[team]["positive_axis"] = !referee.allyOnPositiveHalf();
  //    }

  //    packet[team]["name"] = referee.teams_info[team_id].name;
  //    packet[team]["score"] = referee.teams_info[team_id].score;
  //    packet[team]["timeout"]["remaincount"] = referee.teams_info[team_id].timeout_remaining_count;
  //    packet[team]["timeout"]["remaining_time"] = referee.teams_info[team_id].timeout_remaining_time;
  //    packet[team]["goalkeeper_number"] = referee.teams_info[team_id].goalkeeper_number;

  //    packet[team]["cards"]["yellow"] = referee.teams_info[team_id].yellow_cards_count;
  //    for (uint i = 0; i < referee.teams_info[team_id].yellow_card_times.size(); ++i)
  //    {
  //      packet[team]["cards"]["yellow"]["time"][i] = referee.teams_info[team_id].yellow_card_times.at(i);
  //    }
  //    packet[team]["cards"]["red"] = referee.teams_info[team_id].red_cards_count;
  //    packet[team]["fouls"] = referee.teams_info[team_id].foul_counter;
  //    packet[team]["max_allowed_bots"] = referee.teams_info[team_id].max_allowed_bots;

  for (int team = 0; team < 2; team++)
  {
    std::string team_side = team == 0 ? "ally" : "opponent";

    for (int rid = 0; rid < ai::Config::NB_OF_ROBOTS_BY_TEAM; rid++)
    {
      const data::Robot& current_robot = GlobalDataSingleThread::singleton_.robots_[team][rid];
      const rhoban_geometry::Point& robot_position =
          current_robot.getMovement().linearPosition(GlobalDataSingleThread::singleton_.ai_data_.time);

      packet[team_side][rid]["x"] = robot_position.getX();
      packet[team_side][rid]["y"] = robot_position.getY();
      packet[team_side][rid]["orientation"] =
          current_robot.getMovement().angularPosition(GlobalDataSingleThread::singleton_.ai_data_.time).value();
      packet[team_side][rid]["is_present"] = current_robot.isActive();
      packet[team_side][rid]["id"] = 0;

      if (!ai::Config::is_in_simulation)
      {
        // Activate electronics.
        packet[team_side][rid]["electronics"]["voltage"] = current_robot.electronics.voltage;
        packet[team_side][rid]["electronics"]["cap_volt"] = current_robot.electronics.cap_volt;
      }
    }
  }

  //    // robots informations
  //    for (int rid = 0; rid < ai::Config::NB_OF_ROBOTS_BY_TEAM; rid++)
  //    {
  //      const data::Robot& current_robot = GlobalDataSingleThread::singleton_.robots_[team_id][rid];
  //      const rhoban_geometry::Point& robot_position =
  //          current_robot.getMovement().linearPosition(GlobalDataSingleThread::singleton_.ai_data_.time);

  //      packet[team][rid]["number"] = current_robot.id;
  //      packet[team][rid]["time"] = current_robot.getMovement().lastTime();

  //      packet[team][rid]["position"]["x"] = robot_position.getX();
  //      packet[team][rid]["position"]["y"] = robot_position.getY();
  //      packet[team][rid]["position"]["orientation"] =
  //          current_robot.getMovement().angularPosition(GlobalDataSingleThread::singleton_.ai_data_.time).value();

  //      packet[team][rid]["velocity"]["x"] =
  //          current_robot.getMovement().linearVelocity(GlobalDataSingleThread::singleton_.ai_data_.time).getX();
  //      packet[team][rid]["velocity"]["y"] =
  //          current_robot.getMovement().linearVelocity(GlobalDataSingleThread::singleton_.ai_data_.time).getY();
  //      packet[team][rid]["velocity"]["theta"] =
  //          current_robot.getMovement().angularVelocity(GlobalDataSingleThread::singleton_.ai_data_.time).value();

  //      packet[team][rid]["last_control"]["time"] = 0;
  //      packet[team][rid]["last_control"]["velocity"]["x"] = 0;
  //      packet[team][rid]["last_control"]["velocity"]["y"] = 0;
  //      packet[team][rid]["last_control"]["velocity"]["theta"] = 0;

  //      packet[team][rid]["radius"] = ai::Config::robot_radius;

  //      if (ally_info)
  //      {
  //        packet[team][rid]["color"] = (ai::Config::we_are_blue) ? blue_color : yellow_color;
  //      }
  //      else
  //      {
  //        packet[team][rid]["color"] = (!ai::Config::we_are_blue) ? blue_color : yellow_color;
  //      }
  //      packet[team][rid]["is_present"] = current_robot.isActive();

  //      packet[team][rid]["behavior"] = ai_->getRobotBeheviorOf(rid);
  //      packet[team][rid]["strategy"] = current_robot.isActive();

  //      if (!ai::Config::is_in_simulation)
  //      {
  //        // Activate electronics.
  //        packet[team][rid]["electronics"]["alive"] = current_robot.robotOk();
  //        packet[team][rid]["electronics"]["voltage"] = current_robot.electronics.voltage;
  //        packet[team][rid]["electronics"]["cap_volt"] = current_robot.electronics.cap_volt;
  //        packet[team][rid]["electronics"]["ir_trigered"] = current_robot.infraRed();
  //        packet[team][rid]["electronics"]["odometry"]["x"] = current_robot.electronics.xpos;
  //        packet[team][rid]["electronics"]["odometry"]["y"] = current_robot.electronics.ypos;
  //        packet[team][rid]["electronics"]["odometry"]["orientation"] = current_robot.electronics.ang;
  //        packet[team][rid]["electronics"]["errors"]["driver"] = current_robot.driverError();
  //      }
  // }
  //}
}  // namespace viewer

Json::Value ViewerCommunication::gameInformations()
{
  Json::Value packet;

  // std::string team_color = team == 0 ? color_ally : color_opponent;

  packet["informations"]["simulation"] = ai::Config::is_in_simulation;
  packet["informatons"]["color_ally"] = ai::Config::we_are_blue;

  return packet;
}

}  // namespace viewer
}  // namespace rhoban_ssl
