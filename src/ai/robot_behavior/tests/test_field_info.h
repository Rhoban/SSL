#include <robot_behavior/robot_behavior.h>

#pragma once

namespace rhoban_ssl
{
namespace robot_behavior
{
namespace tests
{
class TestFieldInfo : public RobotBehavior
{
public:
  TestFieldInfo();

  // RobotBehavior interface
public:
  void update(double time, const data::Robot& robot, const data::Ball& ball);
  Control control() const;

  // RobotBehavior interface
public:
  annotations::Annotations getAnnotations() const;
};

}  // namespace tests
}  // namespace robot_behavior
}  // namespace rhoban_ssl
