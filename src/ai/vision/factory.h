#pragma once

#include "robot_position_filter.h"

namespace rhoban_ssl
{
namespace Vision
{
class Factory
{
public:
  static std::pair<rhoban_geometry::Point, ContinuousAngle>
  filter(int robot_id, const SSL_DetectionRobot& robotFrame, Ai::Team team_color, bool ally,
         const std::map<int, SSL_DetectionFrame>& camera_detections, bool& orientation_is_defined,
         const Vision::VisionData& old_vision_data, Vision::Part_of_the_field part_of_the_field_used);
};

};  // namespace Vision
};  // namespace rhoban_ssl
