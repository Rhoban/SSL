/*
    This file is part of SSL.

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

#include "vision_data.h"
#include <assert.h>
#include <debug.h>
#include <VisionClient.h>
#include <data.h>
#include <chrono>

using namespace rhoban_geometry;
using namespace rhoban_utils;

namespace rhoban_ssl
{
namespace vision
{
VisionDataSingleThread VisionDataSingleThread::singleton_;

VisionDataSingleThread::VisionDataSingleThread()
{
  for (unsigned int i = 0; i < ai::Config::NB_CAMERAS; ++i)
  {
    last_camera_detection_[i].frame_number_ = 0;
  }
  for (Team team = 0; team < 2; team++)
    for (int rid = 0; rid < ai::Config::NB_OF_ROBOTS_BY_TEAM; rid++)
      Data::get()->robots[team][rid].id = rid;
}

VisionDataSingleThread::~VisionDataSingleThread()
{
}

///////////////////////////////////////////////////////////////////////////////////////////////////

BallDetection::BallDetection() : confidence_(-1), camera_(nullptr)
{
}

void BallDetection::operator=(const SSL_DetectionBall& b)
{
  confidence_ = b.confidence();
  x_ = b.x();
  y_ = b.y();
  z_ = b.z();
  pixel_x_ = b.pixel_x();
  pixel_y_ = b.pixel_y();
  area_ = b.area();
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void RobotDetection::operator=(const SSL_DetectionRobot& r)
{
  confidence_ = r.confidence();  // set to -1 if not valid
  x_ = r.x();
  y_ = r.y();
  if ((has_orientation_ = r.has_orientation()) == true)
    orientation_ = r.orientation();
  pixel_x_ = r.pixel_x();
  pixel_y_ = r.pixel_y();
  if ((has_height_ = r.has_height()) == true)
    height_ = r.height();
  if ((has_id_ = r.has_robot_id()) == true)
    robot_id_ = r.robot_id();
}

RobotDetection::RobotDetection() : camera_(nullptr), confidence_(-1)
{
}

///////////////////////////////////////////////////////////////////////////////////////////////////

CameraDetectionFrame::CameraDetectionFrame()
  : inverted(false), t_capture_(-1.0), t_sent_(-1.0), frame_number_(0), camera_id_(-1)
{
  for (auto& r : allies_)
    r.camera_ = this;
  for (auto& r : opponents_)
    r.camera_ = this;
  for (auto& b : balls_)
    b.camera_ = this;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

bool VisionDataTerminalPrinter::runTask()
{
  static int counter = 0;
  counter += 1;
  printf("\033[2J\033[1;1H");
  printf("%d\n", counter);
  printf("%d\n", VisionDataGlobal::singleton_.last_packets_.size());
  // VisionDataGlobal::singleton_.packets_buffer_.size());
  auto& field = Data::get()->field;
  printf("Field is present : \n");
  printf("\t %f x %f \n", field.field_width_, field.field_length_);

  for (int camera = 0; camera < ai::Config::NB_CAMERAS; ++camera)
  {
    auto& cam = VisionDataSingleThread::singleton_.last_camera_detection_[camera];
    printf("CAMERA %d (%d): \n", camera, cam.frame_number_);
    printf("\t time: %lf / %lf \n", cam.t_capture_, cam.t_sent_);
    int nballs = 0;
    for (int i = 0; i < ai::Config::MAX_BALLS_DETECTED_PER_CAMERA; ++i)
      if (cam.balls_[i].confidence_ >= 0)
        nballs += 1;
    int nallies = 0;
    for (int i = 0; i < ai::Config::NB_OF_ROBOTS_BY_TEAM; ++i)
      if (cam.allies_[i].confidence_ >= 0)
        nallies += 1;
    int nbopponents = 0;
    for (int i = 0; i < ai::Config::NB_OF_ROBOTS_BY_TEAM; ++i)
      if (cam.opponents_[i].confidence_ >= 0)
        nbopponents += 1;
    printf("\t balls  : %d ", nballs);
    for (auto& i : cam.balls_)
      if (i.confidence_ >= 0)
        printf(" %f (%f,%f) ", i.confidence_, i.x_, i.y_);
    printf("\n");
    printf("\t blues  : %d ", nallies);
    for (auto& i : cam.allies_)
      if (i.confidence_ >= 0)
        printf(" %f (%f,%f) ", i.confidence_, i.x_, i.y_);
    printf("\n");
    printf("\t yellow : %d ", nbopponents);
    for (auto& i : cam.opponents_)
      if (i.confidence_ >= 0)
        printf(" %f (%f,%f) ", i.confidence_, i.x_, i.y_);
    printf("\n");
  }

  if (Data::get()->ball.isActive())
    printf("BALL is at %f %f\n", Data::get()->ball.movement_sample[0].linear_position.x,
           Data::get()->ball.movement_sample[0].linear_position.y);
  else
  {
    printf("BALL is not found\n");
  }

  for (int team = 0; team < 2; ++team)
  {
    printf("team %d\n", team);
    for (int robot = 0; robot < ai::Config::NB_OF_ROBOTS_BY_TEAM; ++robot)
    {
      if (Data::get()->robots[team][robot].isActive())
        printf("\t%d : (%f;%f;%f) ", robot, Data::get()->robots[team][robot].movement_sample[0].linear_position.x,
               Data::get()->robots[team][robot].movement_sample[0].linear_position.y,
               Data::get()->robots[team][robot].movement_sample[0].angular_position.value());
    }
    printf("\n");
  }
  return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

bool ChangeReferencePointOfView::runTask()
{
  if (Data::get()->referee.allyOnPositiveHalf())
  {
    for (uint cam_id = 0; cam_id < ai::Config::NB_CAMERAS; cam_id++)
    {
      if (!VisionDataSingleThread::singleton_.last_camera_detection_[cam_id].inverted)
      {
        for (uint ball_id = 0; ball_id < ai::Config::MAX_BALLS_DETECTED_PER_CAMERA; ball_id++)
        {
          struct BallDetection& ball =
              VisionDataSingleThread::singleton_.last_camera_detection_[cam_id].balls_[ball_id];
          ball.x_ *= -1.f;
          ball.y_ *= -1.f;
        }

        for (uint robot_id = 0; robot_id < ai::Config::NB_OF_ROBOTS_BY_TEAM; robot_id++)
        {
          struct RobotDetection& ally_robot =
              VisionDataSingleThread::singleton_.last_camera_detection_[cam_id].allies_[robot_id];
          ally_robot.x_ *= -1.f;
          ally_robot.y_ *= -1.f;

          if (ally_robot.has_orientation_)
          {
            ally_robot.orientation_ = fmodf32(ally_robot.orientation_ + M_PIf32, M_PIf32 * 2.0f);
          }

          struct RobotDetection& opponent_robot =
              VisionDataSingleThread::singleton_.last_camera_detection_[cam_id].opponents_[robot_id];
          opponent_robot.x_ *= -1.f;
          opponent_robot.y_ *= -1.f;

          if (opponent_robot.has_orientation_)
          {
            opponent_robot.orientation_ = fmodf32(opponent_robot.orientation_ + M_PIf32, M_PIf32 * 2.0f);
          }
        }
        VisionDataSingleThread::singleton_.last_camera_detection_[cam_id].inverted = true;
      }
    }
  }
  return true;
}

}  // namespace vision
}  // namespace rhoban_ssl
