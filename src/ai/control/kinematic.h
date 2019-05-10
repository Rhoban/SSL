#pragma once

#include <ai_data.h>

namespace rhoban_ssl
{
namespace control
{
class Kinematic
{
public:
  Kinematic();

  struct WheelsSpeed
  {
    double frontLeft;
    double frontRight;
    double backLeft;
    double backRight;
  };

  /**
   * @brief Computes the kinematics for the robot.
   * X and Y speed are [m/s], theta speed is [rad/s]
   * It returns the wheel speed in [turn/s]
   */
  WheelsSpeed compute(double x, double y, double theta) const;

  /**
   * @brief load robot informations
   *
   * - robot radius
   * - wheel radius
   * - front wheels angle
   * - rear wheels angle
   * @param ai_data
   */
  void load(const ai::AiData& ai_data);

private:
  bool init_;
  double robot_radius_;
  double wheel_radius_;

  double front_left_x_;
  double front_left_y_;
  double front_right_x_;
  double front_right_y_;
  double rear_left_x_;
  double rear_left_y_;
  double rear_right_x_;
  double rear_right_y_;
};

}  // namespace control
}  // namespace rhoban_ssl
