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
#include "vision_data.h"

using namespace rhoban_geometry;
using namespace rhoban_utils;

namespace rhoban_ssl
{
SslGeometryPacketAnalyzer::SslGeometryPacketAnalyzer() : field_done_(false), camera_done_(false)
{
  // setup field and camera to default values
  // data::Field& field = GlobalDataSingleThread::singleton_.field_;
}

bool SslGeometryPacketAnalyzer::runTask()
{
  for (auto i = rhoban_ssl::VisionDataGlobal::singleton_.last_packets_.begin();
       i != rhoban_ssl::VisionDataGlobal::singleton_.last_packets_.end();)
  {
    if ((*i)->has_geometry())
    {
      DEBUG("geometry packet found");
      // process geometry data and update global
      auto& geometry = (*i)->geometry();
      if ((field_done_ == false) && (geometry.has_field()))
      {
        data::Field& field = GlobalDataSingleThread::singleton_.field_;
        field.field_length_ = geometry.field().field_length() / 1000.0;
        field.field_width_ = geometry.field().field_width() / 1000.0;
        field.goal_width_ = geometry.field().goal_width() / 1000.0;
        field.goal_depth_ = geometry.field().goal_depth() / 1000.0;
        field.boundary_width_ = geometry.field().boundary_width() / 1000.0;
        for (int i = 0; i < geometry.field().field_lines_size(); i++)
        {
          if (geometry.field().field_lines(i).name() == "LeftFieldLeftPenaltyStretch")
          {
            field.penalty_area_depth_ = std::abs(double(geometry.field().field_lines(i).p1().x()) -
                                                 double(geometry.field().field_lines(i).p2().x())) /
                                        1000.0;
            field.penalty_area_width_ = std::abs(2 * double(geometry.field().field_lines(i).p1().y())) / 1000.0;
          }
        }

        //! TODO: Update ALL information in field (corner...)

        // XXX: Receive other data?
        field_done_ = true;
      }
      if ((camera_done_ == false) && (geometry.calib_size() > 0))
      {  // update camera relative informations...
        camera_done_ = true;
      }
      // remove packet as we treated it
      i = rhoban_ssl::VisionDataGlobal::singleton_.last_packets_.erase(i);
    }
    else
    {
      ++i;
    }
  }
  return !(field_done_ && camera_done_);
}

bool DetectionPacketAnalyzer::runTask()
{
  for (auto i = rhoban_ssl::VisionDataGlobal::singleton_.last_packets_.begin();
       i != rhoban_ssl::VisionDataGlobal::singleton_.last_packets_.end();)
  {
    if ((*i)->has_detection())
    {
      auto& frame = (*i)->detection();
      vision::CameraDetectionFrame& current =
          vision::VisionDataSingleThread::singleton_.last_camera_detection_[frame.camera_id()];
      //
      if (frame.frame_number() > current.frame_number_)
      {
        current.frame_number_ = frame.frame_number();
        current.t_sent_ = frame.t_sent();
        current.t_capture_ = frame.t_capture();
        current.camera_id_ = int(frame.camera_id());
        // invalidate previous data
        for (auto& i : current.balls_)
          i.confidence_ = -1;
        for (auto& i : current.allies_)
          i.confidence_ = -1;
        for (auto& i : current.opponents_)
          i.confidence_ = -1;
        for (int i = 0; i < frame.balls_size(); ++i)
          current.balls_[i] = frame.balls(i);
        if (ai::Config::we_are_blue)
        {
          for (int i = 0; i < frame.robots_blue_size(); ++i)
            current.allies_[i] = frame.robots_blue(i);
          for (int i = 0; i < frame.robots_yellow_size(); ++i)
            current.opponents_[i] = frame.robots_yellow(i);
        }
        else
        {
          for (int i = 0; i < frame.robots_blue_size(); ++i)
            current.opponents_[i] = frame.robots_blue(i);
          for (int i = 0; i < frame.robots_yellow_size(); ++i)
            current.allies_[i] = frame.robots_yellow(i);
        }
      }
      i = rhoban_ssl::VisionDataGlobal::singleton_.last_packets_.erase(i);
    }
  }
  return true;
}

UpdateRobotInformation::UpdateRobotInformation(vision::PartOfTheField part_of_the_field_used)
  : part_of_the_field_used_(part_of_the_field_used)
{
}

bool UpdateRobotInformation::runTask()
{
  // go throught cameras and look for robots

  vision::RobotDetection* detections[2][ai::Config::NB_OF_ROBOTS_BY_TEAM][ai::Config::NB_CAMERAS];
  for (int team = 0; team < 2; ++team)
    for (int r = 0; r < ai::Config::NB_OF_ROBOTS_BY_TEAM; ++r)
      for (int c = 0; c < ai::Config::NB_CAMERAS; ++c)
        detections[team][r][c] = nullptr;

  for (int camera_id = 0; camera_id < ai::Config::NB_CAMERAS; ++camera_id)
  {
    auto& camera = vision::VisionDataSingleThread::singleton_.last_camera_detection_[camera_id];
    for (auto& r : camera.allies_)
    {
      if (r.confidence_ < 0)
        continue;
      if (not(objectCoordonateIsValid(r.x_ / 1000.0, r.y_ / 1000.0, part_of_the_field_used_)))
        continue;
      if (r.robot_id_ >= ai::Config::NB_OF_ROBOTS_BY_TEAM)
        continue;
      int i = 0;
      while (detections[Ally][r.robot_id_][i] != nullptr)
        i += 1;
      detections[Ally][r.robot_id_][i] = &r;
    }
    for (auto& r : camera.opponents_)
    {
      if (r.confidence_ < 0)
        continue;
      if (not(objectCoordonateIsValid(r.x_ / 1000.0, r.y_ / 1000.0, part_of_the_field_used_)))
        continue;
      if (r.robot_id_ >= ai::Config::NB_OF_ROBOTS_BY_TEAM)
        continue;
      int i = 0;
      while (detections[Opponent][r.robot_id_][i] != nullptr)
        i += 1;
      detections[Opponent][r.robot_id_][i] = &r;
    }
  }

  // now we have all informations by robots
  for (int team = 0; team < 2; ++team)
    for (int robot = 0; robot < ai::Config::NB_OF_ROBOTS_BY_TEAM; ++robot)
    {
      bool present = false;
      for (uint i = 0; i < ai::Config::NB_CAMERAS; ++i) {
        if( detections[team][robot][i] != nullptr) {
          present = true;
          break;
        }
      }
      if( present ) {
        vision::TimedPosition position = vision::Factory::filter(detections[team][robot]);
        if (position.time_ > 0)
        {
          if (position.orientation_is_defined_)
            GlobalDataSingleThread::singleton_.robots_[team][robot].update(position.time_, position.position_.linear,
                                                                           position.position_.angular);
          else
            GlobalDataSingleThread::singleton_.robots_[team][robot].update(position.time_, position.position_.linear);
        }
      } else {
        // robot is not present in vision
      }
    }
  return true;
}

UpdateBallInformation::UpdateBallInformation(vision::PartOfTheField part_of_the_field_used)
  : part_of_the_field_used_(part_of_the_field_used)
{
}

bool UpdateBallInformation::runTask()
{
  int nballs = 0;
  Point pos(0, 0);
  double tmin = std::numeric_limits<double>::max(), tmax = -std::numeric_limits<double>::min(), t;
  // parse all cameras and balls
  for (auto& c : vision::VisionDataSingleThread::singleton_.last_camera_detection_)
    for (auto& b : c.balls_)
    {
      if (b.confidence_ > 0)
      {
        pos += Point(double(b.x_) / 1000.0, double(b.y_) / 1000.0);
        t = b.camera_->t_capture_;
        tmin = std::min(tmin, t);
        tmax = std::max(tmax, t);
        nballs += 1;
      }
    }
  if (nballs > 0)
  {
    pos = pos / (double)nballs;
    GlobalDataSingleThread::singleton_.ball_.update((tmin + tmax) / 2.0, pos);
  }
  return true;
}

}  // namespace rhoban_ssl
