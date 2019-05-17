#pragma once

#include <VisionClient.h>
#include <math/continuous_angle.h>
#include "vision_data.h"

namespace rhoban_ssl
{
namespace vision
{
enum PartOfTheField
{
  POSIVE_HALF_FIELD,
  NEGATIVE_HALF_FIELD,
  ALL_FIELD
};

bool objectCoordonateIsValid(double x, double y, PartOfTheField part_of_the_field_used);

class RobotPositionFilter
{
public:
  static std::pair<rhoban_geometry::Point, ContinuousAngle>
  averageFilter(int robot_id, const SSL_DetectionRobot& robot_frame, bool ally,
                const std::map<int, SSL_DetectionFrame>& camera_detections, bool& orientation_is_defined,
                vision::PartOfTheField part_of_the_field_used);

  static std::pair<rhoban_geometry::Point, ContinuousAngle>
  exponentialDegressionFilter(int robot_id, const SSL_DetectionRobot& robot_frame, bool ally,
                              const std::map<int, SSL_DetectionFrame>& camera_detections, bool& orientation_is_defined);

  static std::pair<rhoban_geometry::Point, ContinuousAngle>
  noFilter(int robot_id, const SSL_DetectionRobot& robot_frame, bool ally,
           const std::map<int, SSL_DetectionFrame>& camera_detections, bool& orientation_is_defined);
};

};  // namespace vision
};  // namespace rhoban_ssl
