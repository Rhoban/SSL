#pragma once

#include "robot_position_filter.h"

namespace rhoban_ssl
{
namespace vision
{
class Factory
{
public:
  static std::pair<rhoban_geometry::Point, ContinuousAngle>
  filter(int robot_id, const SSL_DetectionRobot& robot_frame, ai::Team team_color, bool ally,
         const std::map<int, SSL_DetectionFrame>& camera_detections, bool& orientation_is_defined,
         const vision::VisionData& old_vision_data, vision::PartOfTheField part_of_the_field_used);
};

};  // namespace vision
};  // namespace rhoban_ssl
