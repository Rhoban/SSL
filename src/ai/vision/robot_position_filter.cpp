#include "robot_position_filter.h"
#include "data.h"

namespace rhoban_ssl
{
namespace vision
{
#define ERROR_FIELD 0.1

bool objectCoordonateIsValid(double x, double y, vision::PartOfTheField part_of_the_field_used)
{
  switch (part_of_the_field_used)
  {
    case PartOfTheField::POSIVE_HALF_FIELD:
    {
      return x > ERROR_FIELD;
    }
    break;
    case PartOfTheField::NEGATIVE_HALF_FIELD:
    {
      return x < ERROR_FIELD;
    }
    break;
    case PartOfTheField::ALL_FIELD:
    {
      return true;
    }
    break;
    default:;
  }
  return false;
}

std::pair<rhoban_geometry::Point, ContinuousAngle>
RobotPositionFilter::averageFilter(int robot_id, const SSL_DetectionRobot& robot_frame, bool ally,
                                   const std::map<int, SSL_DetectionFrame>& camera_detections,
                                   bool& orientation_is_defined, PartOfTheField part_of_the_field_used)
{
  int n_linear = 0;
  int n_angular = 0;
  rhoban_geometry::Point linear_average(0.0, 0.0);
  ContinuousAngle angular_average(0.0);
  double sina = 0.0;
  double cosa = 0.0;
  for (const std::pair<int, SSL_DetectionFrame>& elem : camera_detections)
  {
    // int camera_id = elem.first;
    const SSL_DetectionFrame& detection = elem.second;

    const google::protobuf::RepeatedPtrField<SSL_DetectionRobot>* robots;
    if (ai::Config::we_are_blue)
    {
      robots = &detection.robots_blue();
    }
    else
    {
      robots = &detection.robots_yellow();
    }
    for (auto robot : *robots)
    {
      if (!objectCoordonateIsValid(robot.x() / 1000.0, robot.y() / 1000.0, part_of_the_field_used))
      {
        continue;
      }
      if (robot.has_robot_id() and (robot.robot_id() == static_cast<unsigned int>(robot_id)))
      {
        linear_average += rhoban_geometry::Point(robot.x() / 1000.0, robot.y() / 1000.0);
        n_linear++;
        if (robot.has_orientation())
        {
          // angular_average += ContinuousAngle(
          //   robot.orientation()
          //   );
          sina += sin(robot.orientation());
          cosa += cos(robot.orientation());

          n_angular++;
        }
        break;
      }
    }
  }
  if (n_angular == 0)
  {
    orientation_is_defined = false;
    return { linear_average * (1.0 / n_linear), ContinuousAngle(0.0) };
  }
  else
  {
    orientation_is_defined = true;
    angular_average = atan2(sina, cosa);

    return { linear_average * (1.0 / n_linear), angular_average };
  }
}

std::pair<rhoban_geometry::Point, ContinuousAngle> RobotPositionFilter::exponentialDegressionFilter(
    int robot_id, const SSL_DetectionRobot& robot_frame, bool ally,
    const std::map<int, SSL_DetectionFrame>& camera_detections, bool& orientation_is_defined)
{
  double n_linear = 0;
  double n_angular = 0;
  rhoban_geometry::Point linear_average(0.0, 0.0);
  ContinuousAngle angular_average(0.0);
  for (const std::pair<int, SSL_DetectionFrame>& elem : camera_detections)
  {
    // int camera_id = elem.first;
    const SSL_DetectionFrame& detection = elem.second;

    const google::protobuf::RepeatedPtrField<SSL_DetectionRobot>* robots;
    const MovementSample& old_robot_movement =
        GlobalDataSingleThread::singleton_.robots_[Ally][robot_id].movement_sample;
    if (ai::Config::we_are_blue)
    {
      robots = &detection.robots_blue();
    }
    else
    {
      robots = &detection.robots_yellow();
    }
    for (auto robot : *robots)
    {
      if (robot.has_robot_id() and (robot.robot_id() == static_cast<unsigned int>(robot_id)))
      {
        double x = old_robot_movement.linearPosition().getX();
        double alpha = .5;
        double coef = (((x < 0 and robot.x() / 1000.0 < 0) or (x > 0 and robot.x() / 1000.0 > 0)) ?
                           1 - std::exp(-alpha * std::fabs(x)) :
                           std::exp(-alpha * std::fabs(x)));
        linear_average += (rhoban_geometry::Point(robot.x() / 1000.0, robot.y() / 1000.0) * coef);
        n_linear += coef;
        if (robot.has_orientation())
        {
          angular_average += (ContinuousAngle(robot.orientation()) * coef);
          n_angular += coef;
        }
        break;
      }
    }
  }
  if (n_angular == 0)
  {
    orientation_is_defined = false;
    return { linear_average * (1.0 / n_linear), ContinuousAngle(0.0) };
  }
  else
  {
    orientation_is_defined = true;
    return { linear_average * (1.0 / n_linear), angular_average * (1.0 / n_angular) };
  }
}

std::pair<rhoban_geometry::Point, ContinuousAngle>
RobotPositionFilter::noFilter(int robot_id, const SSL_DetectionRobot& robot_frame, bool ally,
                              const std::map<int, SSL_DetectionFrame>& camera_detections, bool& orientation_is_defined)
{
  orientation_is_defined = robot_frame.has_orientation();
  return { rhoban_geometry::Point(robot_frame.x() / 1000.0, robot_frame.y() / 1000.0),
           robot_frame.has_orientation() ? ContinuousAngle(robot_frame.orientation()) : ContinuousAngle(0.0) };
}

}  // namespace vision
}  // namespace rhoban_ssl
