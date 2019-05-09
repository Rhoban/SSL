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

#pragma once

#include <map>
#include <rhoban_geometry/point.h>
#include <math/continuous_angle.h>
#include <rhoban_utils/timing/time_stamp.h>
#include <physic/movement_sample.h>
#include <iostream>
#include <list>
#include "config.h"
#include <execution_manager.h>

#include <messages_robocup_ssl_wrapper.pb.h>

namespace rhoban_ssl
{
namespace vision
{
static const int history_size = 10;
// static const int Robots = 16;

// typedef enum { Ally = 0, Opponent = 1 } Team;

typedef int Team;
const int Ally = 0;
const int Opponent = 1;

struct Object
{
  MovementSample movement;

  bool present;
  int id;
  rhoban_utils::TimeStamp last_update;

  void update(double time, const rhoban_geometry::Point& linear_position, const rhoban_utils::Angle& angular_position);
  void update(double time, const rhoban_geometry::Point& linear_position, const ContinuousAngle& angular_position);
  void update(double time, const rhoban_geometry::Point& linear_position);

  double age() const;
  bool isOk() const;
  bool isTooOld() const;

  // private:
  // Object(const Object& o);
  // void operator=(const Object&);

public:
  Object();
  void checkAssert(double time) const;
  virtual ~Object();
};

std::ostream& operator<<(std::ostream& out, const Object& object);

struct Robot : Object
{
};
struct Ball : Object
{
};

struct Field
{
  bool present;
  float fieldLength;
  float fieldWidth;
  float goalWidth;
  float goalDepth;
  float boundaryWidth;
  float penaltyAreaDepth;
  float penaltyAreaWidth;

  Field();
};
std::ostream& operator<<(std::ostream& out, const Field& field);

class VisionData
{
public:
  VisionData();

  std::map<Team, std::map<int, Robot>> robots;
  Ball ball;
  Field field;

  void checkAssert(double time) const;

  friend std::ostream& operator<<(std::ostream& out, const rhoban_ssl::vision::VisionData& vision);
};

std::ostream& operator<<(std::ostream& out, const VisionData& vision);

struct CameraDetectionFrame;

struct BallDetection
{
  float confidence_;  // set to -1 if not valid
  float x_;
  float y_;
  float z_;
  float pixel_x_;
  float pixel_y_;
  unsigned int area_;
  CameraDetectionFrame* camera_;

  void operator=(const SSL_DetectionBall&);
  BallDetection();
};

struct RobotDetection
{
  CameraDetectionFrame* camera_;
  float confidence_;  // set to -1 if not valid
  float x_;
  float y_;
  float orientation_;
  float pixel_x_;
  float pixel_y_;

  float height_;
  unsigned int robot_id_;

  bool has_orientation_;
  bool has_id_;
  bool has_height_;

  void operator=(const SSL_DetectionRobot&);
  RobotDetection();
};

struct CameraDetectionFrame
{
  double t_capture_;
  double t_sent_;
  unsigned int frame_number_;
  unsigned int camera_id_;
  struct BallDetection balls_[ai::Config::MAX_BALLS_DETECTED];
  struct RobotDetection allies_[ai::Config::NB_OF_ROBOTS_BY_TEAM];
  struct RobotDetection opponents_[ai::Config::NB_OF_ROBOTS_BY_TEAM];
  CameraDetectionFrame();
};

class VisionDataSingleThread
{
private:
  // avoid copy:
  VisionDataSingleThread(const VisionDataSingleThread&);
  // void operator=(const VisionDataSingleThread&);

public:
  Field field_;
  CameraDetectionFrame last_camera_detection_[ai::Config::NB_CAMERAS];
  Robot robots_[2][ai::Config::NB_OF_ROBOTS_BY_TEAM];
  Ball ball_;
  VisionDataSingleThread();
  ~VisionDataSingleThread();
};

class VisionDataTerminalPrinter : public Task
{
public:
  virtual bool runTask(void);
};

}  // namespace vision
}  // namespace rhoban_ssl
