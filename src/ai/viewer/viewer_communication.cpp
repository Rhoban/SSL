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
#include <viewer_server.h>
#include <manager/factory.h>
#include <core/collection.h>
#include <annotations/annotations.h>

namespace rhoban_ssl
{
namespace viewer
{
ViewerCommunication::ViewerCommunication(ai::AI* ai) : ai_(ai), last_sending_time_(Data::get()->time.now())
{
}

bool ViewerCommunication::runTask()
{
  if (ViewerDataGlobal::get().client_connected)
  {
    processIncomingPackets();

    if (Data::get()->time.now() - last_sending_time_ > sending_delay)
    {
      sendViewerPackets();
      last_sending_time_ = Data::get()->time.now();
    }
  }
  return true;
}

void ViewerCommunication::processIncomingPackets()
{
  std::queue<Json::Value> incoming_packets = viewer::ViewerDataGlobal::get().received_packets.getAndclear();

  while (!incoming_packets.empty())
  {
    Json::Value viewer_packet = incoming_packets.front();

    if (!viewer_packet["emergency"].isNull())
    {
      Data::get()->commander->emergency();
    }
    else if (!viewer_packet["set_packets_per_second"].isNull())
    {
      double new_delay = viewer_packet["set_packets_per_second"].asInt();
      if (new_delay > 0)
        sending_delay = 1 / new_delay;
    }
    else if (!viewer_packet["set_manager"].isNull())
    {
      if (ai_ != nullptr)
        ai_->setManager(viewer_packet["set_manager"].asString());
    }
    else if (!viewer_packet["start_manager"].isNull())
    {
      if (ai_ != nullptr)
        ai_->startManager();
    }
    else if (!viewer_packet["stop_manager"].isNull())
    {
      if (ai_ != nullptr)
        ai_->stopStrategyManager();
    }
    else if (!viewer_packet["halt_bot"].isNull())
    {
      if (ai_ != nullptr)
        ai_->haltRobot(viewer_packet["halt_bot"].asUInt());
    }
    else if (!viewer_packet["enable_bot"].isNull())
    {
      if (ai_ != nullptr)
        ai_->enableRobot(viewer_packet["enable_bot"].asUInt(), true);
    }
    else if (!viewer_packet["disable_bot"].isNull())
    {
      if (ai_ != nullptr)
        ai_->enableRobot(viewer_packet["disable_bot"].asUInt(), false);
    }
    else if (!viewer_packet["control_bot"].isNull())
    {
      processBotsControlBot(viewer_packet["control_bot"]);
    }
    else if (!viewer_packet["set_strategy"].isNull())
    {
      std::vector<int> robot_numbers;
      for (uint i = 0; i < viewer_packet["set_strategy"]["bots"].size(); ++i)
      {
        robot_numbers.push_back(viewer_packet["set_strategy"]["bots"][i].asInt());
      }
      if (ai_)
        ai_->setStrategyManuallyOf(robot_numbers, viewer_packet["set_strategy"]["name"].asString());
    }
    else if (!viewer_packet["place_bot"].isNull())
    {
      bool ally = viewer_packet["place_bot"]["ally"].asBool();
      Data::get()->commander->moveRobot(ally, viewer_packet["place_bot"]["number"].asUInt(),
                                        viewer_packet["place_bot"]["position"]["x"].asDouble(),
                                        viewer_packet["place_bot"]["position"]["y"].asDouble(),
                                        viewer_packet["place_bot"]["position"]["orientation"].asDouble());
    }
    else if (!viewer_packet["place_ball"].isNull())
    {
      Data::get()->commander->moveBall(viewer_packet["place_ball"]["position"]["x"].asDouble(),
                                       viewer_packet["place_ball"]["position"]["y"].asDouble(),
                                       viewer_packet["place_ball"]["speed"]["x"].asDouble(),
                                       viewer_packet["place_ball"]["speed"]["y"].asDouble());
    }
    else if (!viewer_packet["scan"].isNull())
    {
      if (ai_ != nullptr)
        ai_->scan();
    }
    else if (!viewer_packet["play_music"].isNull())
    {
      note n = parseNoteFromJson(viewer_packet["play_music"]);

      DEBUG("robot_number " << n.robot_number << std::endl
                            << "robot_number " << n.robot_number << std::endl
                            << "frequency " << n.frequency << std::endl
                            << "intrument " << n.instrument << std::endl
                            << "duration " << n.duration_in_ms << std::endl
                            << std::endl);
      if (Data::get()->commander->real_ != nullptr)
      {
        Data::get()->commander->real_->addMusicPacket(n.robot_number, parseNoteToMusicPacket(n));
      }
    }
    else
    {
      DEBUG("Invalid viewer packet");
      assert(false);
    }
    incoming_packets.pop();
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
  viewer::ViewerDataGlobal::get().packets_to_send.push(annotationsPacket());
}

Json::Value ViewerCommunication::fieldPacket()
{
  Json::Value packet;
  const data::Field& field = Data::get()->field;

  packet["field"]["length"] = field.field_length;
  packet["field"]["width"] = field.field_width;
  packet["field"]["boundary_width"] = field.boundary_width;
  packet["field"]["goal"]["width"] = field.goal_width;
  packet["field"]["goal"]["depth"] = field.goal_depth;
  packet["field"]["penalty_area"]["width"] = field.penalty_area_width;
  packet["field"]["penalty_area"]["depth"] = field.penalty_area_depth;
  packet["field"]["circle"]["x"] = field.circle_center.getCenter().getX();
  packet["field"]["circle"]["y"] = field.circle_center.getCenter().getY();
  packet["field"]["circle"]["radius"] = field.circle_center.getRadius();

  return packet;
}

Json::Value ViewerCommunication::ballPacket()
{
  Json::Value packet;

  const rhoban_geometry::Point& ball_position = Data::get()->ball.getMovement().linearPosition(Data::get()->time.now());
  packet["ball"]["position"]["x"] = ball_position.getX();
  packet["ball"]["position"]["y"] = ball_position.getY();

  const Vector2d& ball_velocity = Data::get()->ball.getMovement().linearVelocity(Data::get()->time.now());
  packet["ball"]["velocity"]["x"] = ball_velocity.getX();
  packet["ball"]["velocity"]["y"] = ball_velocity.getY();

  packet["ball"]["radius"] = ai::Config::ball_radius;

  return packet;
}

Json::Value ViewerCommunication::teamsPacket()
{
  Json::Value packet;
  double time = Data::get()->time.now();
  const data::Referee& referee = Data::get()->referee;

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
      packet["teams"][team]["positive_axis"] = !Data::get()->referee.allyOnPositiveHalf();
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
      const data::Robot& current_robot = Data::get()->robots[team_id][rid];
      const rhoban_geometry::Point& robot_position = current_robot.getMovement().linearPosition(time);

      packet["teams"][team]["bots"][rid]["number"] = current_robot.id;
      packet["teams"][team]["bots"][rid]["time"] = current_robot.getMovement().lastTime();

      packet["teams"][team]["bots"][rid]["position"]["x"] = robot_position.getX();
      packet["teams"][team]["bots"][rid]["position"]["y"] = robot_position.getY();
      packet["teams"][team]["bots"][rid]["position"]["orientation"] =
          current_robot.getMovement().angularPosition(Data::get()->time.now()).value();

      packet["teams"][team]["bots"][rid]["velocity"]["x"] =
          current_robot.getMovement().linearVelocity(Data::get()->time.now()).getX();
      packet["teams"][team]["bots"][rid]["velocity"]["y"] =
          current_robot.getMovement().linearVelocity(Data::get()->time.now()).getY();
      packet["teams"][team]["bots"][rid]["velocity"]["theta"] =
          current_robot.getMovement().angularVelocity(Data::get()->time.now()).value();

      // TO REMOVE
      packet["teams"][team]["bots"][rid]["last_control"]["time"] = 0;
      packet["teams"][team]["bots"][rid]["last_control"]["velocity"]["x"] = 0;
      packet["teams"][team]["bots"][rid]["last_control"]["velocity"]["y"] = 0;
      packet["teams"][team]["bots"][rid]["last_control"]["velocity"]["theta"] = 0;

      packet["teams"][team]["bots"][rid]["radius"] = ai::Config::robot_radius;
      packet["teams"][team]["bots"][rid]["is_present"] = current_robot.isActive();

      if (ally_info)
      {
        packet["teams"][team]["bots"][rid]["color"] = (ai::Config::we_are_blue) ? blue_color : yellow_color;
        packet["teams"][team]["bots"][rid]["behavior"] = ai_ == nullptr ? "noai" : ai_->getRobotBehaviorOf(rid);
        packet["teams"][team]["bots"][rid]["strategy"] = ai_ == nullptr ? "noai" : ai_->getStrategyOf(rid);

        if (!ai::Config::is_in_simulation)
        {
          // Activate electronics.
          packet["teams"][team]["bots"][rid]["electronics"]["alive"] = current_robot.isOk();
          packet["teams"][team]["bots"][rid]["electronics"]["voltage"] = current_robot.electronics.voltage;
          packet["teams"][team]["bots"][rid]["electronics"]["cap_volt"] = current_robot.electronics.cap_volt;
          packet["teams"][team]["bots"][rid]["electronics"]["ir_triggered"] = current_robot.infraRed();
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

  packet["referee"]["stage"]["value"] = Data::get()->referee.getCurrentStageName();
  packet["referee"]["stage"]["remaining_time"] = Data::get()->referee.stage_time_left;
  packet["referee"]["state"] = Data::get()->referee.getCurrentStateName();

  return packet;
}

Json::Value ViewerCommunication::informationsPacket()
{
  Json::Value packet;

  packet["informations"]["simulation"] = ai::Config::is_in_simulation;
  packet["informations"]["packets_per_second"] = 1. / Data::get()->time.now() - last_sending_time_;

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
  packet["ai"]["managers"]["current"]["name"] = ai_ == nullptr ? "noai" : ai_->getCurrentManager().get()->name();

  for (uint i = 0; ai_ != nullptr && i < ai_->getCurrentManager().get()->getAvailableStrategies().size(); ++i)
  {
    packet["ai"]["managers"]["current"]["strategies_used"][i]["name"] =
        ai_->getCurrentManager().get()->getAvailableStrategies().at(i);
  }

  // we send all strategies in manual manager
  for (uint i = 0; ai_ != nullptr && i < ai_->getManualManager().get()->getAvailableStrategies().size(); ++i)
  {
    std::string strat_name = ai_->getManualManager().get()->getAvailableStrategies().at(i);
    packet["ai"]["strategies"][i]["name"] = strat_name;
    packet["ai"]["strategies"][i]["bots_required"] = ai_->getManualManager().get()->getStrategy(strat_name).minRobots();
  }
  return packet;
}

void ViewerCommunication::processBotsControlBot(const Json::Value& packet)
{
  uint robot_number = packet["number"].asUInt();

  Control& manual_ctrl = Data::get()->shared_data.final_control_for_robots[robot_number].control;

  if (!manual_ctrl.ignore)
  {
    Data::get()->shared_data.final_control_for_robots[robot_number].is_manually_controled_by_viewer = true;

    manual_ctrl.linear_velocity[0] = packet["speed"]["x"].asDouble();
    manual_ctrl.linear_velocity[1] = packet["speed"]["y"].asDouble();
    manual_ctrl.angular_velocity = packet["speed"]["theta"].asDouble();

    manual_ctrl.kick = packet["kicker"]["kick"].asBool();
    manual_ctrl.chip_kick = packet["kicker"]["chip_kick"].asBool();
    manual_ctrl.kick_power = packet["kicker"]["power"].asFloat();
    manual_ctrl.charge = packet["charge"].asBool();

    manual_ctrl.spin = packet["spin"].asBool();

    manual_ctrl.tare_odom = packet["tare_odometry"].asBool();
  }
}

Json::Value ViewerCommunication::annotationsPacket()
{
  if (ai_ == nullptr)
    return Json::Value();
  return ai_->getAnnotations();
}

ViewerCommunication::note ViewerCommunication::parseNoteFromJson(const Json::Value& packet)
{
  note n;
  n.robot_number = packet["number"].asInt();
  n.instrument = packet["instrument"].asString();
  n.frequency = static_cast<u_int16_t>(std::round(packet["frequency"].asDouble()));
  n.duration_in_ms = static_cast<u_int16_t>(packet["duration"].asDouble());

  return n;
}

packet_music ViewerCommunication::parseNoteToMusicPacket(const ViewerCommunication::note& note)
{
  packet_music n;

  if (note.frequency == 0.0)
  {
    n.instrument = 0;
  }
  else
  {
    n.instrument = SOUND_ON;

    if (note.instrument == "beeper")
    {
      n.instrument |= BEEPER;
    }
    if (note.instrument == "kick")
    {
      n.instrument |= KICK;
    }

    if (note.instrument == "chip_kick")
    {
      n.instrument |= CHIP_KICK;
    }

    if (note.instrument == "dribbler")
    {
      n.instrument |= DRIBBLER;
    }

    n.duration = note.duration_in_ms;
    //    n.duration = 1000;
    n.note = note.frequency;
  }
  return n;
}
}  // namespace viewer
}  // namespace rhoban_ssl
