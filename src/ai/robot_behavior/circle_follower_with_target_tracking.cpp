#include "circle_follower_with_target_tracking.h"

namespace rhoban_ssl
{
namespace robot_behavior
{
CircleFollowerWithTargetTracking::CircleFollowerWithTargetTracking(ai::AiData& ai_data)
  : RobotBehavior(ai_data)
{
  initTime(ai_data.time, ai_data.dt);
}

void CircleFollowerWithTargetTracking::update(double time, const ai::Robot& robot,
                                              const ai::Ball& ball)
{
  // At First, we update time and update potition from the abstract class robot_behavior.
  // DO NOT REMOVE THAT LINE
  RobotBehavior::updateTimeAndPosition(time, robot, ball);

  RobotControlWithTargetTrackingAndCircleFollowing::update(time, robot.getMovement().linearPosition(time),
                                                           robot.getMovement().linearVelocity(time),
                                                           robot.getMovement().angularPosition(time));
}

Control CircleFollowerWithTargetTracking::control() const
{
  return getLimitedControl();
}

annotations::Annotations CircleFollowerWithTargetTracking::getAnnotations() const
{
  annotations::Annotations annotations;
  bool dashed = true;

  annotations.addArrow(RobotBehavior::linearPosition(), linear_position_of_the_target_, "red", dashed);

  annotations.addArrow(RobotBehavior::linearPosition(),
                       RobotBehavior::linearPosition() +
                           Vector2d(std::cos(RobotBehavior::angularPosition().value()), std::sin(RobotBehavior::angularPosition().value())),
                       "red", not dashed);
  //    annotations.addArrow(
  //        linear_position(), m_positionToFollow, "magenta", dashed
  //    );

  Control ctrl = control();
  annotations.addArrow(RobotBehavior::linearPosition(), RobotBehavior::linearPosition() + ctrl.linear_velocity, "orange", not dashed);
  return annotations;
}
}  // namespace robot_behavior
}  // namespace rhoban_ssl
