#include "test_field_info.h"
#include <iomanip>

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
  const rhoban_geometry::Point& ball_position = ball.getMovement().linearPosition(time);
  const data::Field field = Data::get()->field;

  printf("\e[1;1H\e[2J");
  std::cout << "---------------------------------------" << std::endl;
  std::cout << std::setw(8) << "Ball position : " << ball_position << std::endl;
  std::cout << std::setw(8) << "Robot position : " << robot.getMovement().linearPosition(time) << std::endl;

  if (field.isInside(ball_position))
    std::cout << std::setw(8) << "Ball inside the field" << std::endl;

  for (int team = 0; team < 2; ++team)
  {
    std::string desc = (team == Ally) ? "ally" : "opponent";
    if (field.getPenaltyArea(team).isInside(ball_position))
      std::cout << std::setw(8) << "Ball inside the " << desc << " penalty area" << std::endl;
  }
}

Control TestFieldInfo::control() const
{
  return Control();
}

annotations::Annotations TestFieldInfo::getAnnotations() const
{
  rhoban_ssl::annotations::Annotations annotations;

  for (int team = 0; team < 2; ++team)
  {
    std::string color = (team == Ally) ? "green" : "red";

    annotations.addBox(Data::get()->field.getPenaltyArea(team), color);
    annotations.addCross(Data::get()->field.getCenterHalfField(team), color);

    annotations.addCross(Data::get()->field.getGoal(team).pole_left, color);
    annotations.addCross(Data::get()->field.getGoal(team).pole_right, color);
    annotations.addCross(Data::get()->field.getGoal(team).goal_center, color);
  }

  for (uint i = 0; i < 4; ++i)
  {
    annotations.addCross(Data::get()->field.getCorner(i), "black");
    annotations.addCross(Data::get()->field.getQuarterCenter(i), "black");
  }

  annotations.addCross(Data::get()->field.centerMark(), "black");
  return annotations;
}

}  // namespace tests
}  // namespace robot_behavior
}  // namespace rhoban_ssl
