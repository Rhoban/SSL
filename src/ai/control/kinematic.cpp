#include "kinematic.h"

namespace rhoban_ssl
{
namespace control
{
Kinematic::Kinematic() : init_(false)
{
}

Kinematic::WheelsSpeed Kinematic::compute(double x, double y, double theta) const
{
  assert(init_);

  // This is a duplication of the computation in the firmware of the mainboard
  WheelsSpeed wheels_speed;

  wheels_speed.frontLeft = (front_left_x_ * x + front_left_y_ * y + robot_radius_ * theta) / (2 * M_PI * wheel_radius_);
  wheels_speed.frontRight =
      (front_right_x_ * x + front_right_y_ * y + robot_radius_ * theta) / (2 * M_PI * wheel_radius_);
  wheels_speed.backLeft = (rear_left_x_ * x + rear_left_y_ * y + robot_radius_ * theta) / (2 * M_PI * wheel_radius_);
  wheels_speed.backRight = (rear_right_x_ * x + rear_right_y_ * y + robot_radius_ * theta) / (2 * M_PI * wheel_radius_);

  return wheels_speed;
}

void Kinematic::load(const ai::AiData& ai_data)
{
  init_ = true;

  double front_angle = rhoban_utils::deg2rad(ai_data.constants.front_wheel_angle);
  double rear_angle = rhoban_utils::deg2rad(ai_data.constants.front_wheel_angle);

  robot_radius_ = ai_data.constants.robot_radius;
  wheel_radius_ = ai_data.constants.wheel_radius;

  front_left_x_ = -sin(front_angle);
  front_left_y_ = -cos(front_angle);

  front_right_x_ = -sin(-front_angle);
  front_right_y_ = -cos(-front_angle);

  rear_left_x_ = -sin(rear_angle);
  rear_left_y_ = -cos(rear_angle);

  rear_right_x_ = -sin(-rear_angle);
  rear_right_y_ = -cos(-rear_angle);
}

}  // namespace control
}  // namespace rhoban_ssl
