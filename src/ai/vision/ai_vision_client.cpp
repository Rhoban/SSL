/*
  This file is part of SSL.

  Copyright 2018 Gréagoire Passault (gregoire.passault@u-bordeaux.fr)
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

#include <iostream>
#include "ai_vision_client.h"
#include <debug.h>
#include "factory.h"
#include <core/print_collection.h>
#include "print_protobuf.h"

using namespace rhoban_geometry;
using namespace rhoban_utils;

namespace rhoban_ssl
{
AIVisionClient::AIVisionClient(Data& shared_data, ai::Team myTeam, bool simulation,
                               vision::PartOfTheField part_of_the_field)
  : VisionClient(simulation), shared_data_(shared_data), part_of_the_field_used_(part_of_the_field), my_team_(myTeam)
{
}

AIVisionClient::AIVisionClient(Data& shared_data, ai::Team myTeam, bool simulation, std::string addr, std::string port,
                               std::string sim_port, vision::PartOfTheField part_of_the_field)
  : VisionClient(simulation, addr, port, sim_port)
  , shared_data_(shared_data)
  , part_of_the_field_used_(part_of_the_field)
  , my_team_(myTeam)
{
}

void AIVisionClient::setRobotPos(ai::Team team, int id, double x, double y, double orientation)
{
  DataFromAi data_from_ai;
  shared_data_ >> data_from_ai;

  my_team_ = data_from_ai.team_color;

  rhoban_ssl::vision::Team visionTeam = rhoban_ssl::vision::Ally;
  if (team != my_team_)
  {
    visionTeam = rhoban_ssl::vision::Opponent;
  }

  mutex.lock();
  vision::Robot& robot = vision_data_.robots.at(visionTeam).at(id);
  double t = robot.movement.time() + 0.01;
  Angle angle(rad2deg(orientation));
  robot.update(t, Point(x, y), angle);
  mutex.unlock();

  shared_data_ << vision_data_;
  old_vision_data_ = vision_data_;
}

void AIVisionClient::packetReceived()
{
  DataFromAi data_from_ai;
  shared_data_ >> data_from_ai;

  my_team_ = data_from_ai.team_color;
  // Retrieving field dimensions
  auto geometry = data_.geometry();
  if (geometry.has_field())
  {
    vision_data_.field.present = true;
    vision_data_.field.fieldLength = geometry.field().field_length() / 1000.0;
    vision_data_.field.fieldWidth = geometry.field().field_width() / 1000.0;
    vision_data_.field.goalWidth = geometry.field().goal_width() / 1000.0;
    vision_data_.field.goalDepth = geometry.field().goal_depth() / 1000.0;
    vision_data_.field.boundaryWidth = geometry.field().boundary_width() / 1000.0;
    for (int i = 0; i < geometry.field().field_lines_size(); i++)
    {
      if (geometry.field().field_lines(i).name() == "LeftFieldLeftPenaltyStretch")
      {
        vision_data_.field.penaltyAreaDepth =
            std::fabs(geometry.field().field_lines(i).p1().x() - geometry.field().field_lines(i).p2().x()) / 1000.0;
        vision_data_.field.penaltyAreaWidth = std::fabs(2 * geometry.field().field_lines(i).p1().y()) / 1000.0;
      }
    }
    // XXX: Receive other data?
  }

  const SSL_DetectionFrame& detection = data_.detection();

  // DEBUG("DETECTION : " << detection);

  // Update the historic of camera detections
  auto it = camera_detections_.find(detection.camera_id());
  if (it == camera_detections_.end() or it->second.t_capture() < detection.t_capture())
  {
    camera_detections_[detection.camera_id()] = detection;
  }

  // Ball informations
  if (detection.balls().size())
  {
    if (!vision_data_.ball.present || vision_data_.ball.age() > 1)
    {
      // std::cerr<<"IF"<<std::endl;
      // If the ball is outdated (> 1s) or not present, taking the first
      // one in the frame

      double ball_is_detected = -1.0;
      for (auto ball : detection.balls())
      {
        double x = ball.x() / 1000.0;
        double y = ball.y() / 1000.0;
        if (objectCoordonateIsValid(x, y, part_of_the_field_used_))
        {
          ball_is_detected = detection.t_sent();
          vision_data_.ball.update(detection.t_sent(), Point(x, y));  // TODO HACK : IL FAUT METTRE t_send() ?
          ball_camera_detections_[detection.camera_id()] = { detection.t_sent(), Point(x, y) };
          break;
        }
        // std::cerr<<"CONFIDENCE IF ("<<detection.camera_id()<<"): "<<ball.confidence()<<" t:
        // "<<detection.t_sent()<<std::endl;
      }
      ball_camera_detections_[detection.camera_id()].first = ball_is_detected;
      // ball_camera_detections[detection.camera_id()].first = -1.0;
    }
    else
    {
      // std::cerr<<"ELSE"<<std::endl;
      // Else, we accept the ball which is the nearest from the previous one
      // we already had
      bool hasBall = false;
      Point bestBall(-99999, -99999);
      double nearest = 99999;

      for (auto ball : detection.balls())
      {
        double x = ball.x() / 1000.0;
        double y = ball.y() / 1000.0;
        // std::cerr<<"CONFIDENCE ELSE ("<<detection.camera_id()<<"): "<<ball.confidence()<<" t:
        // "<<detection.t_sent()<<std::endl;
        if (not(objectCoordonateIsValid(x, y, part_of_the_field_used_)

                    ))
        {
          continue;
        }
        Point pos(x, y);

        double distance = pos.getDist(vision_data_.ball.movement[0].linear_position);

        if (!hasBall || distance < nearest)
        {
          nearest = distance;
          bestBall = pos;
        }
        // std::cout<<"BALL: "<<pos<<" dist: "<<distance<<" nearest: "<<bestBall<<" dist: "<<nearest<<std::endl;
      }
      if (nearest < 99999)
        hasBall = true;

      if (hasBall)
      {
        // std::cout<<"HASBALL"<<std::endl;
        ball_camera_detections_[detection.camera_id()] = { detection.t_sent(), bestBall };
        auto final_ball = averageFilter(bestBall, ball_camera_detections_, part_of_the_field_used_);
        // std::cout<<"HASBALL final: "<<final_ball<<std::endl;
        vision_data_.ball.update(detection.t_sent(), final_ball);  // TODO HACK : IL FAUT METTRE t_send() ?
      }
      else
      {
        // ball_camera_detections[detection.camera_id()].first = false;
        ball_camera_detections_[detection.camera_id()].first = -1.0;
      }
    }
  }
  else
  {
    // ball_camera_detections[detection.camera_id()].first = false;
    ball_camera_detections_[detection.camera_id()].first = -1.0;
  }

  // We set to not present all robot that is too old
  for (unsigned int i = 0; i < vision_data_.robots.at(vision::Team::Ally).size(); i++)
  {
    vision::Robot& robot = vision_data_.robots.at(vision::Team::Ally).at(i);
    if (robot.isTooOld())
    {
      robot.present = false;
    }
  }
  for (unsigned int i = 0; i < vision_data_.robots.at(vision::Team::Opponent).size(); i++)
  {
    vision::Robot& robot = vision_data_.robots.at(vision::Team::Opponent).at(i);
    if (robot.isTooOld())
    {
      robot.present = false;
    }
  }

  // Robots informations
  for (auto robot : detection.robots_blue())
  {
    updateRobotInformation(detection, robot, my_team_ == ai::Blue, ai::Blue);
  }
  for (auto robot : detection.robots_yellow())
  {
    updateRobotInformation(detection, robot, my_team_ == ai::Yellow, ai::Yellow);
  }

  shared_data_ << vision_data_;
}

void AIVisionClient::updateRobotInformation(const SSL_DetectionFrame& detection, const SSL_DetectionRobot& robotFrame,
                                            bool ally, ai::Team team_color)
{
  if (not(objectCoordonateIsValid(robotFrame.x() / 1000.0, robotFrame.y() / 1000.0, part_of_the_field_used_)))
  {
    return;
  }
  if (robotFrame.has_robot_id())
  {
    if (robotFrame.robot_id() < ai::Constants::NB_OF_ROBOTS_BY_TEAM)
    {
      vision::Team team = ally ? vision::Team::Ally : vision::Team::Opponent;
      vision::Robot& robot = vision_data_.robots.at(team).at(robotFrame.robot_id());

      bool orientation_is_defined = false;
      std::pair<rhoban_geometry::Point, ContinuousAngle> position =
          vision::Factory::filter(robotFrame.robot_id(), robotFrame, team_color, ally, camera_detections_,
                                  orientation_is_defined, old_vision_data_, part_of_the_field_used_);
      //                Point position = Point(robotFrame.x()/1000.0, robotFrame.y()/1000.0);

      if (orientation_is_defined)
      {
        Angle orientation(rad2deg(position.second.value()));
        robot.update(detection.t_sent(), position.first, orientation);  // TODO HACK : IL FAUT METTRE t_send() ?
      }
      else
      {
        robot.update(detection.t_sent(), position.first);
      }
    }
    else
    {
      DEBUG("Warnings : Vision have detected a robot with id " << robotFrame.robot_id() << ".");
    }
  }
}

rhoban_geometry::Point AIVisionClient::averageFilter(const rhoban_geometry::Point& new_ball,
                                                      std::map<int,               // CMAERA ID
                                                               std::pair<double,  // camera have found a ball
                                                                         rhoban_geometry::Point  // detecte ball
                                                                         > >& ball_camera_detections,
                                                      vision::PartOfTheField part_of_the_field_used)
{
  int n_linear = 0;
  rhoban_geometry::Point linear_average(0.0, 0.0);
  for (const std::pair<int,                              // CMAERA ID
                       std::pair<double,                 // time capture_t
                                 rhoban_geometry::Point  // detecte ball
                                 > >& elem : ball_camera_detections)
  {
    double ball_is_detected = elem.second.first;  // TODO
    double camera_id = elem.first;

    const rhoban_geometry::Point& ball_pos = elem.second.second;
    // std::cerr<<"DETECTED: "<<ball_is_detected<<std::endl;
    if (ball_is_detected > 0.0)
    {
      // linear_average += rhoban_geometry::Point(
      //   new_ball.getX()/1000.0, new_ball.getY()/1000.0
      //   );

      linear_average += rhoban_geometry::Point(
          // ball_pos.getX()/1000.0, ball_pos.getY()/1000.0
          ball_pos.getX(), ball_pos.getY());

      n_linear++;
    }
    // std::cerr<<"FILTER ("<<camera_id<<"): "<<ball_pos<<std::endl;
  }
  // std::cerr<<std::endl;
  return linear_average * (1.0 / n_linear);
}

}  // namespace rhoban_ssl
