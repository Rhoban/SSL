#include "test_field_info.h"

namespace rhoban_ssl
{
namespace robot_behavior
{
namespace tests
{
TestFieldInfo::TestFieldInfo()
{
}

void TestFieldInfo::update(double time, const data::Robot& robot, const data::Ball& ball)
{
}

Control TestFieldInfo::control() const
{
  return Control();
}

annotations::Annotations TestFieldInfo::getAnnotations() const
{
  rhoban_ssl::annotations::Annotations annotations;

  annotations.addBox(Data::get()->field.getPenaltyArea(Opponent), "red");
  annotations.addBox(Data::get()->field.getPenaltyArea(Ally), "green");

  for (uint i = 0; i < 4; ++i)
  {
    annotations.addCross(Data::get()->field.getCorner(i), "black");
    annotations.addCross(Data::get()->field.getQuarterCenter(i), "black");
  }

  annotations.addCross(Data::get()->field.getCenterHalfField(Ally), "grey");
  annotations.addCross(Data::get()->field.getCenterHalfField(Opponent), "grey");

  annotations.addCross(Data::get()->field.centerMark(), "black");

  annotations.addCross(Data::get()->field.getGoal(Ally).pole_left_, "blue");
  annotations.addCross(Data::get()->field.getGoal(Ally).pole_right_, "blue");
  annotations.addCross(Data::get()->field.getGoal(Ally).goal_center_, "blue");

  annotations.addCross(Data::get()->field.getGoal(Opponent).pole_left_, "red");
  annotations.addCross(Data::get()->field.getGoal(Opponent).pole_right_, "red");
  annotations.addCross(Data::get()->field.getGoal(Opponent).goal_center_, "red");

  return annotations;
}

}  // namespace tests
}  // namespace robot_behavior
}  // namespace rhoban_ssl
