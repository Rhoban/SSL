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
  float v_x_;
  float v_y_;
  float orientation_;
  float angular_speed_;
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
  /**
   * @brief inverted is custom field
   *
   * It is true if we already invert detections.
   */
  bool inverted;

  double t_capture_;
  double t_sent_;
  unsigned int frame_number_;
  int camera_id_;
  struct BallDetection balls_[ai::Config::MAX_BALLS_DETECTED_PER_CAMERA];
  struct RobotDetection allies_[ai::Config::NB_OF_ROBOTS_BY_TEAM];
  struct RobotDetection opponents_[ai::Config::NB_OF_ROBOTS_BY_TEAM];
  CameraDetectionFrame();
};

class VisionDataSingleThread
{
private:
  // avoid copy:
  VisionDataSingleThread(const VisionDataSingleThread&);
  void operator=(const VisionDataSingleThread&);

  friend class UpdateRobotInformation;
  friend class DetectionPacketAnalyzer;
  friend class UpdateBallInformation;

public:
  static VisionDataSingleThread singleton_;

  CameraDetectionFrame last_camera_detection_[ai::Config::NB_CAMERAS];
  VisionDataSingleThread();
  ~VisionDataSingleThread();
};

class ChangeReferencePointOfView : public Task
{
  virtual bool runTask(void);
};

class VisionDataTerminalPrinter : public Task
{
  virtual bool runTask(void);
};

}  // namespace vision
}  // namespace rhoban_ssl
