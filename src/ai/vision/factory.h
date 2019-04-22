#pragma once

#include "robot_position_filter.h"

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
  filter(int robot_id, const SSL_DetectionRobot& robot_frame, ai::Team team_color, bool ally,
         const std::map<int, SSL_DetectionFrame>& camera_detections, bool& orientation_is_defined,
         const vision::VisionData& old_vision_data, vision::PartOfTheField part_of_the_field_used);

  static TimedPosition filter(RobotDetection** robots);
};

};  // namespace vision
};  // namespace rhoban_ssl
