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

#include "robot_position_filter.h"
#include <math/position.h>
#include <math/gsl/gsl_matrix.h>
#include <debug.h>

#define KALMAN_STATE_VECTOR_SIZE 4             // tuple (x,y,theta ; vx, vy, vtheta)
#define KALMAN_STATE_SUBVECTOR_SIZE 2         // subtuple position/speed 
#define KALMAN_OBSERVATION_VECTOR_ELEMENTAR_SIZE 4  // position and speed in base (x, y, theta)

namespace rhoban_ssl
{
namespace vision
{
struct TimedPosition
{
  double time_;
  Position position_;
  bool orientation_is_defined_;
  TimedPosition();
};

class Factory
{
public:
  static std::pair<rhoban_geometry::Point, ContinuousAngle>
  filter(int robot_id, const SSL_DetectionRobot& robot_frame, bool ally,
         const std::map<int, SSL_DetectionFrame>& camera_detections, bool& orientation_is_defined,
         vision::PartOfTheField part_of_the_field_used);

  static TimedPosition filter(RobotDetection** robots);
};

class Kalman
{
public :
  GslMatrix previous_state_ ;
  GslMatrix previous_state_covariance_ ;
  double previous_kalman_execution_time_;

  double kalmanTimePrefilter(CameraDetectionFrame last_camera_detections[ai::Config::NB_CAMERAS]);

  void kalmanSpeedPrefilter(vision::RobotDetection* previous_detections[2][ai::Config::NB_OF_ROBOTS_BY_TEAM][ai::Config::NB_CAMERAS], vision::RobotDetection* new_detections[2][ai::Config::NB_OF_ROBOTS_BY_TEAM][ai::Config::NB_CAMERAS], double dt);

  void disableOrientation(GslMatrix* observation_model, size_t row_offset, size_t col_offset, size_t subvector_length);

  void setupPredictPhaseParams(GslMatrix* physical_model, GslMatrix* process_noise_matrix, double dt);

  void setupUpdatePhaseParams(GslMatrix* observation_model, const int camera_number, GslMatrix* observation_noise_matrix);

  TimedPosition kalmanFilter(RobotDetection** robots, double cadence_time);

  Kalman();
  ~Kalman();
};

};  // namespace vision
};  // namespace rhoban_ssl
