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
#include "AIVisionClient.h"
#include <debug.h>
#include "factory.h"
#include <core/print_collection.h>
#include "print_protobuf.h"

using namespace rhoban_geometry;
using namespace rhoban_utils;

namespace RhobanSSL
{
AIVisionClient::AIVisionClient(Data& shared_data, Ai::Team myTeam, bool simulation,
                               Vision::Part_of_the_field part_of_the_field)
  : VisionClient(simulation), shared_data(shared_data), part_of_the_field_used(part_of_the_field), myTeam(myTeam)
{
}

AIVisionClient::AIVisionClient(Data& shared_data, Ai::Team myTeam, bool simulation, std::string addr, std::string port,
                               std::string sim_port, Vision::Part_of_the_field part_of_the_field)
  : VisionClient(simulation, addr, port, sim_port)
  , shared_data(shared_data)
  , part_of_the_field_used(part_of_the_field)
  , myTeam(myTeam)
{
}

void AIVisionClient::setRobotPos(Ai::Team team, int id, double x, double y, double orientation)
{
  Data_from_ai data_from_ai;
  shared_data >> data_from_ai;

  myTeam = data_from_ai.team_color;

  RhobanSSL::Vision::Team visionTeam = RhobanSSL::Vision::Ally;
  if (team != myTeam)
  {
    visionTeam = RhobanSSL::Vision::Opponent;
  }

  mutex.lock();
  Vision::Robot& robot = visionData.robots.at(visionTeam).at(id);
  double t = robot.movement.time() + 0.01;
  Angle angle(rad2deg(orientation));
  robot.update(t, Point(x, y), angle);
  mutex.unlock();

  shared_data << visionData;
  oldVisionData = visionData;
}

void AIVisionClient::packetReceived()
{
  Data_from_ai data_from_ai;
  shared_data >> data_from_ai;

  myTeam = data_from_ai.team_color;
  // Retrieving field dimensions
  auto geometry = data.geometry();
  if (geometry.has_field())
  {
    visionData.field.present = true;
    visionData.field.fieldLength = geometry.field().field_length() / 1000.0;
    visionData.field.fieldWidth = geometry.field().field_width() / 1000.0;
    visionData.field.goalWidth = geometry.field().goal_width() / 1000.0;
    visionData.field.goalDepth = geometry.field().goal_depth() / 1000.0;
    visionData.field.boundaryWidth = geometry.field().boundary_width() / 1000.0;
    for (int i = 0; i < geometry.field().field_lines_size(); i++)
    {
      if (geometry.field().field_lines(i).name() == "LeftFieldLeftPenaltyStretch")
      {
        visionData.field.penaltyAreaDepth =
            std::fabs(geometry.field().field_lines(i).p1().x() - geometry.field().field_lines(i).p2().x()) / 1000.0;
        visionData.field.penaltyAreaWidth = std::fabs(2 * geometry.field().field_lines(i).p1().y()) / 1000.0;
      }
    }
    // XXX: Receive other data?
  }

  const SSL_DetectionFrame& detection = data.detection();

  // DEBUG("DETECTION : " << detection);

  // Update the historic of camera detections
  auto it = camera_detections.find(detection.camera_id());
  if (it == camera_detections.end() or it->second.t_capture() < detection.t_capture())
  {
    camera_detections[detection.camera_id()] = detection;
  }

  // Ball informations
  if (detection.balls().size())
  {
    if (!visionData.ball.present || visionData.ball.age() > 1)
    {
      // std::cerr<<"IF"<<std::endl;
      // If the ball is outdated (> 1s) or not present, taking the first
      // one in the frame

      double ball_is_detected = -1.0;
      for (auto ball : detection.balls())
      {
        double x = ball.x() / 1000.0;
        double y = ball.y() / 1000.0;
        if (object_coordonate_is_valid(x, y, part_of_the_field_used))
        {
          ball_is_detected = detection.t_sent();
          visionData.ball.update(detection.t_sent(), Point(x, y));  // TODO HACK : IL FAUT METTRE t_send() ?
          ball_camera_detections[detection.camera_id()] = { detection.t_sent(), Point(x, y) };
          break;
        }
        // std::cerr<<"CONFIDENCE IF ("<<detection.camera_id()<<"): "<<ball.confidence()<<" t:
        // "<<detection.t_sent()<<std::endl;
      }
      ball_camera_detections[detection.camera_id()].first = ball_is_detected;
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
        if (not(object_coordonate_is_valid(x, y, part_of_the_field_used)

                    ))
        {
          continue;
        }
        Point pos(x, y);

        double distance = pos.getDist(visionData.ball.movement[0].linear_position);

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
        ball_camera_detections[detection.camera_id()] = { detection.t_sent(), bestBall };
        auto final_ball = average_filter(bestBall, ball_camera_detections, part_of_the_field_used);
        // std::cout<<"HASBALL final: "<<final_ball<<std::endl;
        visionData.ball.update(detection.t_sent(), final_ball);  // TODO HACK : IL FAUT METTRE t_send() ?
      }
      else
      {
        // ball_camera_detections[detection.camera_id()].first = false;
        ball_camera_detections[detection.camera_id()].first = -1.0;
      }
    }
  }
  else
  {
    // ball_camera_detections[detection.camera_id()].first = false;
    ball_camera_detections[detection.camera_id()].first = -1.0;
  }

  // We set to not present all robot that is too old
  for (unsigned int i = 0; i < visionData.robots.at(Vision::Team::Ally).size(); i++)
  {
    Vision::Robot& robot = visionData.robots.at(Vision::Team::Ally).at(i);
    if (robot.is_too_old())
    {
      robot.present = false;
    }
  }
  for (unsigned int i = 0; i < visionData.robots.at(Vision::Team::Opponent).size(); i++)
  {
    Vision::Robot& robot = visionData.robots.at(Vision::Team::Opponent).at(i);
    if (robot.is_too_old())
    {
      robot.present = false;
    }
  }

  // Robots informations
  for (auto robot : detection.robots_blue())
  {
    updateRobotInformation(detection, robot, myTeam == Ai::Blue, Ai::Blue);
  }
  for (auto robot : detection.robots_yellow())
  {
    updateRobotInformation(detection, robot, myTeam == Ai::Yellow, Ai::Yellow);
  }

  shared_data << visionData;
}

void AIVisionClient::updateRobotInformation(const SSL_DetectionFrame& detection, const SSL_DetectionRobot& robotFrame,
                                            bool ally, Ai::Team team_color)
{
  if (not(object_coordonate_is_valid(robotFrame.x() / 1000.0, robotFrame.y() / 1000.0, part_of_the_field_used)))
  {
    return;
  }
  if (robotFrame.has_robot_id())
  {
    if (robotFrame.robot_id() < Ai::Constants::NB_OF_ROBOTS_BY_TEAM)
    {
      Vision::Team team = ally ? Vision::Team::Ally : Vision::Team::Opponent;
      Vision::Robot& robot = visionData.robots.at(team).at(robotFrame.robot_id());

      bool orientation_is_defined = false;
      std::pair<rhoban_geometry::Point, ContinuousAngle> position =
          Vision::Factory::filter(robotFrame.robot_id(), robotFrame, team_color, ally, camera_detections,
                                  orientation_is_defined, oldVisionData, part_of_the_field_used);
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

rhoban_geometry::Point AIVisionClient::average_filter(const rhoban_geometry::Point& new_ball,
                                                      std::map<int,               // CMAERA ID
                                                               std::pair<double,  // camera have found a ball
                                                                         rhoban_geometry::Point  // detecte ball
                                                                         > >& ball_camera_detections,
                                                      Vision::Part_of_the_field part_of_the_field_used)
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

}  // namespace RhobanSSL
