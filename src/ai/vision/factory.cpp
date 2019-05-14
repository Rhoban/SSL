#include "factory.h"

namespace rhoban_ssl
{
namespace vision
{
std::pair<rhoban_geometry::Point, ContinuousAngle>
Factory::filter(int robot_id, const SSL_DetectionRobot& robot_frame, bool ally,
                const std::map<int, SSL_DetectionFrame>& camera_detections, bool& orientation_is_defined, vision::PartOfTheField part_of_the_field_used)
{
  return RobotPositionFilter::averageFilter(robot_id, robot_frame, ally, camera_detections,
                                            orientation_is_defined, part_of_the_field_used);
  // return Robot_position_filter::exponential_degression_filter(
  //    robot_id, robotFrame, team_color, ally, camera_detections, orientation_is_defined, old_vision_data
  //);
  // return Robot_position_filter::no_filter(
  //    robot_id, robotFrame, team_color, ally, camera_detections, orientation_is_defined, old_vision_data
  //);
}

TimedPosition Factory::filter(RobotDetection** robots)
{
  TimedPosition t;
  int i;
  double average_time = 0.0;
  rhoban_geometry::Point linear_average(0.0, 0.0);
  ContinuousAngle angular_average(0.0);
  double sina = 0.0;
  double cosa = 0.0;
  int n_angular = 0;

  for (i = 0; i < ai::Config::NB_CAMERAS && robots[i] != nullptr; ++i)
  {
    average_time += robots[i]->camera_->t_capture_;
    linear_average += rhoban_geometry::Point(robots[i]->x_ / 1000.0, robots[i]->y_ / 1000.0);
    if (robots[i]->has_orientation_)
    {
      sina += sin(robots[i]->orientation_);
      cosa += cos(robots[i]->orientation_);
      n_angular++;
    }
  }
  if (i > 0)
  {
    t.time_ = average_time / (double)i;
    t.position_.linear = linear_average / (double)i;
    if (n_angular == 0)
      t.orientation_is_defined_ = false;
    else
    {
      t.orientation_is_defined_ = true;
      angular_average = atan2(sina, cosa);
      t.position_.angular = angular_average;
    }
  }
  return t;
}

TimedPosition::TimedPosition() : time_(0), orientation_is_defined_(false)
{
}

}  // namespace vision
}  // namespace rhoban_ssl
