#include "factory.h"

namespace rhoban_ssl
{
namespace Vision
{
std::pair<rhoban_geometry::Point, ContinuousAngle>
Factory::filter(int robot_id, const SSL_DetectionRobot& robotFrame, ai::Team team_color, bool ally,
                const std::map<int, SSL_DetectionFrame>& camera_detections, bool& orientation_is_defined,
                const Vision::VisionData& old_vision_data, Vision::PartOfTheField part_of_the_field_used)
{
  return RobotPositionFilter::averageFilter(robot_id, robotFrame, team_color, ally, camera_detections,
                                               orientation_is_defined, old_vision_data, part_of_the_field_used);
  // return Robot_position_filter::exponential_degression_filter(
  //    robot_id, robotFrame, team_color, ally, camera_detections, orientation_is_defined, old_vision_data
  //);
  // return Robot_position_filter::no_filter(
  //    robot_id, robotFrame, team_color, ally, camera_detections, orientation_is_defined, old_vision_data
  //);
}

}  // namespace Vision
}  // namespace rhoban_ssl
