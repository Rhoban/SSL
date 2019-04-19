#include "circle_follower_with_target_tracking.h"

namespace rhoban_ssl
{
namespace robot_behavior
{
CircleFollowerWithTargetTracking::CircleFollowerWithTargetTracking(RhobanSSL::Ai::AiData& ai_data)
  : RhobanSSL::Robot_behavior::RobotBehavior(ai_data)
{
  initTime(ai_data.time, ai_data.dt);
}

void CircleFollowerWithTargetTracking::update(double time, const RhobanSSL::Ai::Robot& robot,
                                              const RhobanSSL::Ai::Ball& ball)
{
  RobotControlWithTargetTrackingAndCircleFollowing::update(time, robot.get_movement().linear_position(time),
                                                           robot.get_movement().linear_velocity(time),
                                                           robot.get_movement().angular_position(time));
}

Control CircleFollowerWithTargetTracking::control() const
{
  return getLimitedControl();
}

RhobanSSLAnnotation::Annotations CircleFollowerWithTargetTracking::get_annotations() const
{
  RhobanSSLAnnotation::Annotations annotations;
  bool dashed = true;

  annotations.addArrow(linear_position(), linear_position_of_the_target_, "red", dashed);

  annotations.addArrow(linear_position(),
                       linear_position() +
                           Vector2d(std::cos(angular_position().value()), std::sin(angular_position().value())),
                       "red", not dashed);

  //    annotations.addArrow(
  //        linear_position(), m_positionToFollow, "magenta", dashed
  //    );

  Control ctrl = control();
  annotations.addArrow(linear_position(), linear_position() + ctrl.linear_velocity, "orange", not dashed);
  return annotations;
}
}  // namespace robot_behavior
}  // namespace rhoban_ssl
