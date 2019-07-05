#pragma once

#include <robot_behavior/robot_behavior.h>

namespace rhoban_ssl
{
namespace robot_behavior
{
namespace test
{
class TestNavigationInsideTheField : public RobotBehavior
{
  // RobotBehavior interface
public:
  void update(double time, const data::Robot& robot, const data::Ball& ball);
  Control control() const;
  annotations::Annotations getAnnotations() const;
};
}  // namespace test
}  // namespace robot_behavior
}  // namespace rhoban_ssl
