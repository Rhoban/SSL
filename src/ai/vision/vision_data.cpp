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
Field::Field()
  : present(false)
  , fieldLength(0.0)
  , fieldWidth(0.0)
  , goalWidth(0.0)
  , goalDepth(0.0)
  , boundaryWidth(0.0)
  , penaltyAreaDepth(0.0)
  , penaltyAreaWidth(0.0)
{
}

void Object::update(double time, const Point& linear_position)
{
  update(time, linear_position, movement[0].angular_position);
}

void Object::update(double time, const Point& linear_position, const Angle& angular_position)
{
  ContinuousAngle angle(movement[0].angular_position);
  angle.setToNearest(angular_position);
  update(time, linear_position, angle);
}

void Object::update(double time, const Point& linear_position, const ContinuousAngle& angular_position)
{
  if (time <= movement.time(0))
  {
    // TODO
    // DEBUG("TODO");
    return;
  }
  last_update = rhoban_utils::TimeStamp::now();
  present = true;

  movement.insert(PositionSample(time, linear_position, angular_position));
}

double Object::age() const
{
  return diffSec(last_update, rhoban_utils::TimeStamp::now());
}

bool Object::isTooOld() const
{
  return not(present) or age() > 4.0;
}

bool Object::isOk() const
{
  return present && age() < 2.0;
}

Object::Object() : movement(history_size), present(false), id(-1), last_update(rhoban_utils::TimeStamp::now())
{
  for (int i = 0; i < history_size; i++)
  {
    movement[i].time = -i;
  }
}

VisionData::VisionData()
{
  field.present = false;

  for (auto team : { Ally, Opponent })
  {
    for (int k = 0; k < ai::Config::NB_OF_ROBOTS_BY_TEAM; k++)
    {
      robots[team][k].id = k;
      if (team == Ally)
      {
        robots[team][k].update(1, Point(-1 - k * 0.3, 3.75));
      }
      else
      {
        robots[team][k].update(1, Point(1 + k * 0.3, 3.75));
      }
      robots[team][k].present = false;
    }
  }
}

void Object::checkAssert(double time) const
{
  assert(not(present) or (movement.time(0) > movement.time(1) and movement.time(1) > movement.time(2)));
  //    assert(
  //        not(present) or ( time > movement.time(0) )
  //    );
}

Object::~Object()
{
}

void VisionData::checkAssert(double time) const
{
  for (auto team : { Ally, Opponent })
  {
    for (int k = 0; k < ai::Config::NB_OF_ROBOTS_BY_TEAM; k++)
    {
      robots.at(team).at(k).checkAssert(time);
    }
  }
  ball.checkAssert(time);
}

std::ostream& operator<<(std::ostream& out, const rhoban_ssl::vision::VisionData& vision)
{
  for (auto team : { rhoban_ssl::vision::Ally, rhoban_ssl::vision::Opponent })
  {
    out << team << " : " << std::endl;
    for (int k = 0; k < ai::Config::NB_OF_ROBOTS_BY_TEAM; k++)
    {
      out << "robot " << k << std::endl;
      out << vision.robots.at(team).at(k);
    }
  }
  out << "ball : " << std::endl;
  out << vision.ball << std::endl;

  return out;
}

std::ostream& operator<<(std::ostream& out, const rhoban_ssl::vision::Object& object)
{
  out << " id : " << object.id << std::endl;
  out << " present : " << object.present << std::endl;
  out << " age : " << object.age() << std::endl;
  out << " lastUpdate : " << object.last_update.getTimeMS() / 1000.0 << std::endl;
  out << " movement : " << object.movement << std::endl;
  return out;
}

std::ostream& operator<<(std::ostream& out, const Field& field)
{
  out << "field -- len. " << field.fieldLength << " , width " << field.fieldWidth;
  return out;
}

VisionDataSingleThread::VisionDataSingleThread(const VisionDataSingleThread&)
{
  throw "Vision Data must not be copied!";
}

VisionDataSingleThread::VisionDataSingleThread()
{
  for (unsigned int i = 0; i < ai::Config::NB_CAMERAS; ++i)
  {
    last_camera_detection_[i].frame_number_ = 0;
  }
}

VisionDataSingleThread::~VisionDataSingleThread()
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

BallDetection::BallDetection() : confidence_(-1), camera_(nullptr)
{
}

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

bool VisionDataTerminalPrinter::runTask()
{
  static int counter = 0;
  counter += 1;
  printf("\033[2J\033[1;1H");
  printf("%d\n", counter);
  printf("%d\n", VisionDataGlobal::singleton_.last_packets_.size());
  // VisionDataGlobal::singleton_.packets_buffer_.size());
  auto& field = GlobalDataSingleThread::singleton_.vision_data_.field_;
  if (field.present)
  {
    printf("Field is present : \n");
    printf("\t %f x %f \n", field.fieldWidth, field.fieldLength);
  }

  for (int camera = 0; camera < ai::Config::NB_CAMERAS; ++camera)
  {
    auto& cam = GlobalDataSingleThread::singleton_.vision_data_.last_camera_detection_[camera];
    printf("CAMERA %d (%d): \n", camera, cam.frame_number_);
    printf("\t time: %lf / %lf \n", cam.t_capture_, cam.t_sent_);
    int nballs = 0;
    for (int i = 0; i < ai::Config::MAX_BALLS_DETECTED; ++i)
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

  if (GlobalDataSingleThread::singleton_.vision_data_.ball_.present)
    printf("BALL is at %f %f\n", GlobalDataSingleThread::singleton_.vision_data_.ball_.movement[0].linear_position.x,
           GlobalDataSingleThread::singleton_.vision_data_.ball_.movement[0].linear_position.y);
  else
  {
    printf("BALL is not found\n");
  }

  for (int team = 0; team < 2; ++team)
  {
    printf("team %d\n", team);
    for (int robot = 0; robot < ai::Config::NB_OF_ROBOTS_BY_TEAM; ++robot)
    {
      if (GlobalDataSingleThread::singleton_.vision_data_.robots_[team][robot].present)
        printf("\t%d : (%f;%f) ", robot,
               GlobalDataSingleThread::singleton_.vision_data_.robots_[team][robot].movement[0].linear_position.x,
               GlobalDataSingleThread::singleton_.vision_data_.robots_[team][robot].movement[0].linear_position.y);
    }
    printf("\n");
  }
  return true;
}

CameraDetectionFrame::CameraDetectionFrame() : t_capture_(-1.0), t_sent_(-1.0), frame_number_(0), camera_id_(-1)
{
  for (auto& r : allies_)
    r.camera_ = this;
  for (auto& r : opponents_)
    r.camera_ = this;
  for (auto& b : balls_)
    b.camera_ = this;
}

}  // namespace vision
}  // namespace rhoban_ssl
